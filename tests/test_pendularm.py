import math
from checks import (Rosbridge, silent, call, close, vector, session, stamp, subscribe, state,
                    trajectory, frozen_plant, integrate, ik_roundtrip, fk)


@silent
def test_01_euler_uses_old_velocity_and_start_time():
    with Rosbridge() as ros:
        result = integrate(ros, 'euler')
        x, v, dt = 1.25, -0.4, 0.125
        for i in range(7):
            t = i * dt
            x, v = x + v*dt, v + (t*t + 3*t - 1)*dt
            close(result['positions'][i+1], x)
            close(result['velocities'][i+1], v)


@silent
def test_02_midpoint_uses_half_time_and_half_velocity():
    with Rosbridge() as ros:
        result = integrate(ros, 'midpoint')
        x, v, dt = 1.25, -0.4, 0.125
        for i in range(7):
            t = i * dt
            x += v*dt + (t*t + 3*t - 1)*dt*dt/2
            v += ((t+dt/2)**2 + 3*(t+dt/2) - 1)*dt
            close(result['positions'][i+1], x)
            close(result['velocities'][i+1], v)


@silent
def test_03_verlet_averages_endpoint_accelerations():
    with Rosbridge() as ros:
        result = integrate(ros, 'verlet')
        x, v, dt = 1.25, -0.4, 0.125
        for i in range(7):
            t = i * dt
            a, b = t*t+3*t-1, (t+dt)**2+3*(t+dt)-1
            x += v*dt + a*dt*dt/2
            v += (a+b)*dt/2
            close(result['positions'][i+1], x)
            close(result['velocities'][i+1], v)


@silent
def test_04_rk4_matches_polynomial_antiderivatives():
    with Rosbridge() as ros:
        result = integrate(ros, 'rk4')
        for i, t in enumerate(result['times']):
            close(result['positions'][i], 1.25 - 0.4*t + t**4/12 + t**3/2 - t*t/2)
            close(result['velocities'][i], -0.4 + t**3/3 + 1.5*t*t - t)


@silent
def test_05_expression_associativity_precedence_and_unary():
    with Rosbridge() as ros:
        for expression, acceleration in [('2^3^2',512), ('8/4/2',1), ('-2^2',4),
                                          ('2+3*4',14), ('(2+3)*4',20), ('0.01 + .5',0.51)]:
            result = integrate(ros, 'rk4', expression, x0=0, v0=0, dt=0.25, steps=2)
            close(result['positions'][-1], acceleration*0.5**2/2)
            close(result['velocities'][-1], acceleration*0.5)


@silent
def test_06_expression_named_functions_and_default_velocity():
    with Rosbridge() as ros:
        result = call(ros, '/arm_sim/integration_step', {'function': 'sin(0)+cos(0)+tan(0)+exp(0)+sqrt(9)+ln(exp(1))+abs(-2)',
                      'x0': -2, 'dt': 0.25, 'steps': 4, 'integrator': 'rk4'})
        vector(result['velocities'], [0,2,4,6,8])
        close(result['positions'][-1], 2)


@silent
def test_07_malformed_expressions_reject_and_recover():
    with Rosbridge() as ros:
        for expression in ['', '(', 'sin(t', 'foo(t)', 't garbage', '1 2', 't@2', 't+']:
            call(ros, '/arm_sim/integration_step', {'function': expression, 'x0': 0, 'dt': 0.1,
                 'steps': 1, 'integrator': 'rk4'}, success=False)
        integrate(ros, 'rk4')


@silent
def test_08_integration_argument_validation_and_recovery():
    with Rosbridge() as ros:
        base = {'function':'t', 'x0':0, 'dt':0.1, 'steps':2, 'integrator':'rk4'}
        for change in [{'dt':0}, {'dt':-0.1}, {'steps':0}, {'steps':-1}, {'steps':1.5},
                       {'integrator':'bogus'}, {'x0':'x'}]:
            call(ros, '/arm_sim/integration_step', dict(base, **change), success=False)
        integrate(ros, 'rk4')


@silent
def test_09_checkpoint_service_does_not_change_live_state():
    with session() as (ros, n):
        subscribe(ros)
        before = state(ros, n)
        for method in ['euler', 'midpoint', 'verlet', 'rk4']:
            integrate(ros, method, x0=123, v0=-9)
        after = state(ros, n)
        for field in ['position', 'velocity', 'effort']:
            vector(after[field], before[field])
        close(stamp(after), stamp(before))


@silent
def test_10_integrator_partial_updates_queries_and_rejection():
    with session() as (ros, n):
        for method in ['euler', 'midpoint', 'verlet', 'rk4']:
            assert call(ros, '/arm_sim/set_integrator', {'method':method})['method'] == method
        values = call(ros, '/arm_sim/set_integrator', {'timestep':0.003})
        close(values['timestep'], 0.003)
        assert values['method'] == 'rk4'
        values = call(ros, '/arm_sim/set_integrator', {'method':'invalid', 'timestep':0.007}, success=False)
        assert values['method'] == 'rk4'
        close(values['timestep'], 0.007)
        values = call(ros, '/arm_sim/set_integrator', {'timestep':0}, success=False)
        close(values['timestep'], 0.007)


@silent
def test_11_parameter_fields_validate_independently_and_echo():
    with session() as (ros, n):
        values = call(ros, '/arm_sim/set_params', {'gravity':2.3, 'masses':[1.2]*n, 'lengths':[0.8]*n})
        assert values == call(ros, '/arm_sim/set_params')
        for bad in [[1]*(n-1), [0]*n, [-1]*n]:
            values = call(ros, '/arm_sim/set_params', {'masses':bad, 'gravity':3.4}, success=False)
            vector(values['masses'], [1.2]*n)
            close(values['gravity'], 3.4)
        values = call(ros, '/arm_sim/set_params', {'gravity':-1, 'lengths':[0.7]*n}, success=False)
        close(values['gravity'], 3.4)
        vector(values['lengths'], [0.7]*n)
        values = call(ros, '/arm_sim/set_params', {'lengths':[0]*n}, success=False)
        vector(values['lengths'], [0.7]*n)


@silent
def test_12_pid_gain_fields_validate_independently_and_echo():
    with session() as (ros, n):
        values = call(ros, '/pid_controller/set_gains', {'kp':[2]*n, 'ki':[3]*n, 'kd':[4]*n})
        assert values == call(ros, '/pid_controller/set_gains')
        for key in ['kp', 'ki', 'kd']:
            old = values[key][:]
            values = call(ros, '/pid_controller/set_gains', {key:[-1]*n}, success=False)
            vector(values[key], old)
            values = call(ros, '/pid_controller/set_gains', {key:[1]*(n-1)}, success=False)
            vector(values[key], old)
        values = call(ros, '/pid_controller/set_gains', {'kp':[-1]*n, 'ki':[0.7]*n}, success=False)
        vector(values['kp'], [2]*n)
        vector(values['ki'], [0.7]*n)


@silent
def test_13_pause_freezes_clock_state_and_keeps_publishing():
    with session() as (ros, n):
        call(ros, '/arm_sim/set_params', {'gravity':9.81})
        subscribe(ros)
        call(ros, '/arm_sim/pause', {'data':False})
        moving = state(ros, n, lambda s: stamp(s)>0.03)
        assert any(abs(v)>0.01 for v in moving['velocity'])
        call(ros, '/arm_sim/pause', {'data':True})
        ros.pending.clear()
        for _ in range(3):
            state(ros,n)
        first = state(ros,n)
        for _ in range(4):
            latest = state(ros,n)
        close(stamp(first),stamp(latest))
        vector(first['position'],latest['position'])
        vector(first['velocity'],latest['velocity'])
        assert call(ros, '/arm_sim/pause')['paused'] is True


@silent
def test_14_reset_clears_motion_old_target_and_pid_integral():
    with session() as (ros, n):
        frozen_plant(ros,n,ki=2)
        trajectory(ros,n,[0.4]*n)
        call(ros,'/arm_sim/pause',{'data':False})
        state(ros,n,lambda s: stamp(s)>0.15)
        call(ros,'/arm_sim/pause',{'data':True})
        values = call(ros,'/arm_sim/reset')
        vector(values['position'],[0]*n)
        vector(values['velocity'],[0]*n)
        assert call(ros,'/pid_controller/enable')['enabled'] is True
        ros.pending.clear()
        call(ros,'/arm_sim/pause',{'data':False})
        latest = state(ros,n,lambda s: stamp(s)>0.03 and max(map(abs,s['effort']))<1e-8)
        vector(latest['position'],[0]*n)


@silent
def test_15_disabled_pid_reports_zero_effort():
    with session() as (ros,n):
        call(ros,'/pid_controller/set_gains',{'kp':[20]*n,'ki':[3]*n,'kd':[5]*n})
        trajectory(ros,n,[0.5]*n,[0.8]*n)
        subscribe(ros)
        call(ros,'/arm_sim/pause',{'data':False})
        latest = state(ros,n,lambda s: stamp(s)>0.03)
        vector(latest['effort'],[0]*n)
        call(ros,'/pid_controller/enable',{'data':True})
        latest = state(ros,n,lambda s:max(map(abs,s['effort']))>0.1)
        call(ros,'/pid_controller/enable',{'data':False})
        state(ros,n,lambda s:max(map(abs,s['effort']))==0)


@silent
def test_16_proportional_term_uses_last_waypoint_per_joint():
    with session() as (ros,n):
        frozen_plant(ros,n,kp=3)
        target = [0.2*(i+1)*(-1)**i for i in range(n)]
        trajectory(ros,n,target,first=[-0.9]*n)
        call(ros,'/arm_sim/pause',{'data':False})
        latest=state(ros,n,lambda s:stamp(s)>0.03)
        vector(latest['effort'],[3*v for v in target],1e-6)


@silent
def test_17_derivative_term_uses_commanded_velocity_and_defaults():
    with session() as (ros,n):
        frozen_plant(ros,n,kd=4)
        desired = [0.3*(-1)**i for i in range(n)]
        trajectory(ros,n,[0]*n,desired)
        call(ros,'/arm_sim/pause',{'data':False})
        latest=state(ros,n,lambda s:stamp(s)>0.03)
        vector(latest['effort'],[4*v for v in desired],1e-6)
        call(ros,'/arm_sim/pause',{'data':True})
        trajectory(ros,n,[0]*n,[])  # wrong-length velocities default to zero
        call(ros,'/arm_sim/pause',{'data':False})
        latest=state(ros,n,lambda s: max(map(abs,s['effort']))<1e-6)
        vector(latest['effort'],[0]*n,1e-6)


@silent
def test_18_integral_accumulates_preserves_gains_and_clears_on_enable():
    with session() as (ros,n):
        frozen_plant(ros,n,ki=2)
        trajectory(ros,n,[0.5]*n)
        call(ros,'/arm_sim/pause',{'data':False})
        first=state(ros,n,lambda s:stamp(s)>0.08)
        last=state(ros,n,lambda s:stamp(s)>0.2)
        delta=stamp(last)-stamp(first)
        vector([a-b for a,b in zip(last['effort'],first['effort'])],[delta]*n,2e-6)
        call(ros,'/arm_sim/pause',{'data':True})
        call(ros,'/pid_controller/set_gains',{'ki':[4]*n})
        ros.pending.clear()
        call(ros,'/arm_sim/pause',{'data':False})
        doubled=state(ros,n,lambda s:stamp(s)>stamp(last)+0.02)
        assert min(doubled['effort'])>1.8*min(last['effort'])
        call(ros,'/arm_sim/pause',{'data':True})
        call(ros,'/pid_controller/enable',{'data':False})
        call(ros,'/pid_controller/enable',{'data':True})
        ros.pending.clear()
        call(ros,'/arm_sim/pause',{'data':False})
        cleared=state(ros,n,lambda s:stamp(s)>stamp(doubled) and max(map(abs,s['effort']))<0.15)
        assert max(map(abs,cleared['effort']))<0.15


@silent
def test_19_pid_converges_to_small_target_without_gravity():
    with session() as (ros,n):
        call(ros,'/arm_sim/set_params',{'masses':[1.0]*n,'lengths':[0.5]*n})
        call(ros,'/pid_controller/set_gains',{'kp':[12]*n,'kd':[3]*n})
        call(ros,'/arm_sim/set_integrator',{'timestep':0.001})
        call(ros,'/pid_controller/enable',{'data':True})
        subscribe(ros)
        target=[0.18*(-1)**i for i in range(n)]
        trajectory(ros,n,target)
        call(ros,'/arm_sim/pause',{'data':False})
        latest=state(ros,n,lambda s:stamp(s)>1.8,timeout=20)
        vector(latest['position'],target,0.08)


@silent
def test_20_zero_gravity_and_zero_torque_preserve_rest():
    with session() as (ros,n):
        subscribe(ros)
        call(ros,'/arm_sim/pause',{'data':False})
        latest=state(ros,n,lambda s:stamp(s)>0.1)
        vector(latest['position'],[0]*n)
        vector(latest['velocity'],[0]*n)
        vector(latest['effort'],[0]*n)


@silent
def test_21_gravity_acceleration_matches_coupled_rigid_rods():
    with session() as (ros,n):
        lengths=[0.7+0.2*i for i in range(n)]
        masses=[1+0.4*i for i in range(n)]
        gravity=4.0
        call(ros,'/arm_sim/set_params',{'gravity':gravity,'lengths':lengths,'masses':masses})
        call(ros,'/arm_sim/set_integrator',{'timestep':0.001})
        # At horizontal rest, each rod's angular inertia is m*l^2/12;
        # COM speed from joint j is the distance from j to that COM.
        matrix=[[0.0]*n for _ in range(n)]
        load=[0.0]*n
        for k in range(n):
            lever=[sum(lengths[j:k])+lengths[k]/2 for j in range(k+1)]
            for j in range(k+1):
                load[j]-=masses[k]*gravity*lever[j]
                for i in range(k+1):
                    matrix[j][i]+=masses[k]*(lever[j]*lever[i]+lengths[k]**2/12)
        # Independent Cholesky solve of SPD M*a=-G.
        lower=[[0.0]*n for _ in range(n)]
        for i in range(n):
            for j in range(i+1):
                x=matrix[i][j]-sum(lower[i][k]*lower[j][k] for k in range(j))
                lower[i][j]=math.sqrt(x) if i==j else x/lower[j][j]
        y=[0.0]*n
        for i in range(n):
            y[i]=(load[i]-sum(lower[i][j]*y[j] for j in range(i)))/lower[i][i]
        acceleration=[0.0]*n
        for i in reversed(range(n)):
            acceleration[i]=(y[i]-sum(lower[j][i]*acceleration[j] for j in range(i+1,n)))/lower[i][i]
        subscribe(ros)
        call(ros,'/arm_sim/pause',{'data':False})
        latest=state(ros,n,lambda s:0.006<stamp(s)<0.04,timeout=3)
        vector([v/stamp(latest) for v in latest['velocity']],acceleration,0.02)


@silent
def test_22_ik_roundtrips_quadrants_and_updated_link_lengths():
    with session() as (ros,n):
        for lengths in [[1]*n,[0.7+0.3*i for i in range(n)]]:
            call(ros,'/arm_sim/set_params',{'lengths':lengths})
            for first in [0.2,1.8,-2.1,-0.7]:
                q=[first,-0.9]+([0.4] if n==3 else [])
                ik_roundtrip(ros,q,lengths)


@silent
def test_23_ik_reach_boundaries_inner_hole_orientation_and_bad_input():
    with session() as (ros,n):
        lengths=[2,0.75]+([0.5] if n==3 else [])
        call(ros,'/arm_sim/set_params',{'lengths':lengths})
        ik_roundtrip(ros,[0]*n,lengths)
        if n==2:
            ik_roundtrip(ros,[0,math.pi],lengths)
            call(ros,'/ik/solve',{'x':0,'y':0},success=False)
        else:
            # Position inside total reach, but desired wrist beyond links 1+2.
            call(ros,'/ik/solve',{'x':3.0,'y':0,'phi':math.pi},success=False)
        for args in [{'x':sum(lengths)+0.2,'y':0}, {'x':0}, {'x':'a','y':0}]:
            call(ros,'/ik/solve',args,success=False)


@silent
def test_24_action_goal_rejection_cancel_and_preemption():
    with session() as (ros,n):
        ros.subscribe('/ik_action/result','std_msgs/String',timeout=2)
        args={'x':n-0.3,'y':0.2,'phi':0,'success_hold':100}
        first=call(ros,'/ik_action/send_goal',args)['goal_id']
        assert isinstance(first,str) and first
        call(ros,'/ik_action/send_goal',{'x':n+10,'y':0},success=False)
        call(ros,'/ik_action/cancel_goal',{'goal_id':'wrong'},success=False)
        second=call(ros,'/ik_action/send_goal',args)['goal_id']
        assert first!=second
        result=ros.next_message('/ik_action/result',timeout=2,match=lambda m:m['goal_id']==first)
        assert result['outcome']=='preempted' and math.isfinite(result['final_distance'])
        call(ros,'/ik_action/cancel_goal',{'goal_id':second})
        result=ros.next_message('/ik_action/result',timeout=2,match=lambda m:m['goal_id']==second)
        assert result['outcome']=='preempted'
        call(ros,'/ik_action/cancel_goal',{},success=False)


@silent
def test_25_trial_start_skip_stop_clear_status():
    with session() as (ros,n):
        ros.subscribe('/ik_trial/status','std_msgs/String',timeout=2)
        call(ros,'/ik_trial/skip',{},success=False)
        call(ros,'/ik_trial/stop',{},success=False)
        call(ros,'/ik_trial/start',{'duration':5,'epsilon':0.05,'success_hold':100})
        running=ros.next_message('/ik_trial/status',timeout=2,match=lambda m:m['running'] is True)
        assert running['targets_reached']==0 and running['target'] is not None
        assert len(running['desired_positions'])==n
        call(ros,'/ik_trial/skip')
        call(ros,'/ik_trial/stop')
        stopped=ros.next_message('/ik_trial/status',timeout=2,match=lambda m:m['running'] is False)
        assert stopped['target'] is None and stopped['error'] is None
        assert stopped['desired_positions'] is None and stopped['action_status']=='idle'
