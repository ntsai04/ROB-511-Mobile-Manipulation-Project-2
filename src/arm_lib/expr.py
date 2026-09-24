"""Safe parser for the integration checkpoint's small expression language.

Expressions become ``float -> float`` callables instead of being passed to
``eval``.  That keeps a service request limited to arithmetic in ``t`` and
makes malformed input an ordinary service error rather than executable code.
"""

import math
from collections.abc import Callable


class Expression:
    def __init__(self, source: str) -> None:
        self.source = source
        self._tree = _Parser(source).parse()

    def __call__(self, t: float) -> float:
        return self._tree(float(t))


def parse_expression(source: str) -> Callable[[float], float]:
    """Parse the restricted arithmetic expression described in the handout."""
    if not isinstance(source, str):
        raise ValueError("function must be a string")
    return Expression(source)


class _Parser:
    """Recursive-descent parser whose AST nodes are ``float -> float`` callables.

    The method order mirrors the precedence table in the handout: sum,
    product, power (right associative), unary minus, and atoms.
    """
    def __init__(self, source: str) -> None:
        self.tokens = self._tokenize(source)
        self.index = 0

    @staticmethod
    def _tokenize(source: str) -> list[tuple[str, str]]:
        import re
        pattern = re.compile(r"\s*(?:(\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|([A-Za-z_][A-Za-z_0-9]*)|(.))")
        tokens: list[tuple[str, str]] = []
        position = 0
        while position < len(source):
            if source[position:].strip() == "":
                break
            match = pattern.match(source, position)
            if match is None:
                raise ValueError("invalid expression")
            position = match.end()
            number, name, other = match.groups()
            if number is not None:
                tokens.append(("number", number))
            elif name is not None:
                tokens.append(("name", name))
            elif other and not other.isspace():
                tokens.append((other, other))
        if not tokens:
            raise ValueError("function is empty")
        tokens.append(("end", ""))
        return tokens

    def _take(self, kind: str) -> str | None:
        if self.tokens[self.index][0] == kind:
            self.index += 1
            return self.tokens[self.index - 1][1]
        return None

    def parse(self) -> Callable[[float], float]:
        value = self._sum()
        if self.tokens[self.index][0] != "end":
            raise ValueError("unexpected trailing input")
        return value

    def _sum(self):
        left = self._product()
        while self.tokens[self.index][0] in ("+", "-"):
            operator = self.tokens[self.index][0]
            self.index += 1
            right = self._product()
            left = _binary(operator, left, right)
        return left

    def _product(self):
        left = self._power()
        while self.tokens[self.index][0] in ("*", "/"):
            operator = self.tokens[self.index][0]
            self.index += 1
            right = self._power()
            left = _binary(operator, left, right)
        return left

    def _power(self):
        left = self._unary()
        if self._take("^") is not None:
            return _binary("^", left, self._power())
        return left

    def _unary(self):
        if self._take("-") is not None:
            operand = self._unary()
            return lambda t: -operand(t)
        return self._atom()

    def _atom(self):
        number = self._take("number")
        if number is not None:
            value = float(number)
            return lambda _t: value
        name = self._take("name")
        if name is not None:
            if name == "t":
                return lambda t: t
            if name not in FUNCTIONS or self._take("(") is None:
                raise ValueError(f"unknown identifier or function syntax: {name}")
            argument = self._sum()
            if self._take(")") is None:
                raise ValueError("expected closing parenthesis")
            function = FUNCTIONS[name]
            return lambda t: _safe_function(function, argument(t))
        if self._take("(") is not None:
            value = self._sum()
            if self._take(")") is None:
                raise ValueError("expected closing parenthesis")
            return value
        raise ValueError("expected expression")


def _safe_function(function: Callable[[float], float], value: float) -> float:
    try:
        return function(value)
    except (ValueError, OverflowError):
        return math.nan


def _binary(operator: str, left, right):
    """Build a numeric binary-expression node that turns domain errors into NaN."""
    def evaluate(t: float) -> float:
        a, b = left(t), right(t)
        try:
            if operator == "+":
                value = a + b
            elif operator == "-":
                value = a - b
            elif operator == "*":
                value = a * b
            elif operator == "/":
                value = a / b
            else:
                value = a ** b
            # Python produces a complex number for (-1) ** 0.5 rather than
            # raising.  The checkpoint's scalar state cannot integrate that,
            # so treat it like every other real-domain error.
            return float(value) if not isinstance(value, complex) else math.nan
        except (ValueError, OverflowError, ZeroDivisionError):
            return math.nan
    return evaluate


FUNCTIONS = {
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "exp": math.exp,
    "sqrt": math.sqrt,
    "ln": math.log,
    "abs": abs,
}
