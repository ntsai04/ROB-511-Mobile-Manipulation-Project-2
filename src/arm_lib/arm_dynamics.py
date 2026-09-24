"""Lagrangian dynamics for a planar serial RR...R arm."""

import math

from .internal import ArmParameters


class ArmDynamics:
    def __init__(self, parameters: ArmParameters) -> None:
        self.parameters = parameters

    def mass_matrix(self, q: list[float]) -> list[list[float]]:
        """Return M(q), derived from the link kinetic energy."""
        n = self.parameters.links
        if len(q) != n:
            raise ValueError("joint vector has wrong length")
        angles = []
        total = 0.0
        for value in q:
            total += value
            angles.append(total)
        matrix = [[0.0 for _ in range(n)] for _ in range(n)]
        for link in range(n):
            jx = [0.0] * n
            jy = [0.0] * n
            for joint in range(link + 1):
                # Every joint before this COM rotates every subsequent segment.
                for segment in range(joint, link + 1):
                    distance = self.parameters.lengths[segment]
                    if segment == link:
                        distance *= 0.5
                    jx[joint] -= distance * math.sin(angles[segment])
                    jy[joint] += distance * math.cos(angles[segment])
            mass = self.parameters.masses[link]
            inertia = mass * self.parameters.lengths[link] ** 2 / 12.0
            for row in range(link + 1):
                for column in range(link + 1):
                    # The rotational Jacobian is one for each upstream joint.
                    matrix[row][column] += mass * (jx[row] * jx[column] + jy[row] * jy[column]) + inertia
        return matrix

    def gravity_load(self, q: list[float]) -> list[float]:
        """Return G(q), the gradient of gravitational potential energy."""
        n = self.parameters.links
        angles = []
        total = 0.0
        for value in q:
            total += value
            angles.append(total)
        result = [0.0] * n
        for joint in range(n):
            for link in range(joint, n):
                height_derivative = 0.0
                for segment in range(joint, link + 1):
                    distance = self.parameters.lengths[segment]
                    if segment == link:
                        distance *= 0.5
                    height_derivative += distance * math.cos(angles[segment])
                result[joint] += self.parameters.masses[link] * self.parameters.gravity * height_derivative
        return result

    def coriolis_load(self, q: list[float], qdot: list[float]) -> list[float]:
        """Return C(q, qdot) qdot using Christoffel symbols."""
        n = self.parameters.links
        if len(qdot) != n:
            raise ValueError("joint velocity vector has wrong length")
        # A centered derivative is compact, general for 2/3 links, and avoids
        # duplicating a fragile symbolic expression for each arm size.
        delta = 1e-6
        derivatives: list[list[list[float]]] = []
        for coordinate in range(n):
            plus, minus = list(q), list(q)
            plus[coordinate] += delta
            minus[coordinate] -= delta
            mp, mm = self.mass_matrix(plus), self.mass_matrix(minus)
            derivatives.append([[(mp[i][j] - mm[i][j]) / (2 * delta)
                                 for j in range(n)] for i in range(n)])
        load = [0.0] * n
        for i in range(n):
            for j in range(n):
                for k in range(n):
                    gamma = 0.5 * (derivatives[k][i][j] + derivatives[j][i][k]
                                   - derivatives[i][j][k])
                    load[i] += gamma * qdot[j] * qdot[k]
        return load

    def acceleration(
        self, q: list[float], qdot: list[float], effort: list[float]
    ) -> list[float]:
        """Solve M(q) qddot = effort - C(q,qdot)qdot - G(q)."""
        n = self.parameters.links
        if len(effort) != n:
            raise ValueError("effort vector has wrong length")
        mass = self.mass_matrix(q)
        coriolis = self.coriolis_load(q, qdot)
        gravity = self.gravity_load(q)
        rhs = [effort[i] - coriolis[i] - gravity[i] for i in range(n)]
        # Gaussian elimination with partial pivoting; M is positive definite
        # for valid physical parameters, but pivoting makes roundoff harmless.
        augmented = [mass[i][:] + [rhs[i]] for i in range(n)]
        for column in range(n):
            pivot = max(range(column, n), key=lambda row: abs(augmented[row][column]))
            if abs(augmented[pivot][column]) < 1e-14:
                raise ValueError("singular mass matrix")
            augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
            scale = augmented[column][column]
            for index in range(column, n + 1):
                augmented[column][index] /= scale
            for row in range(n):
                if row == column:
                    continue
                factor = augmented[row][column]
                for index in range(column, n + 1):
                    augmented[row][index] -= factor * augmented[column][index]
        return [augmented[i][n] for i in range(n)]
