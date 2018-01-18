import sys
from typing import List


def mysum1(nums: List[int]) -> int:
    result = 0

    for num in nums:
        result += num

    print(result)
    return result


def myfib1(n: int) -> None:
    def find_fib(n_: int) -> int:
        if n_ <= 1:
            return n_

        return find_fib(n_ - 1) + find_fib(n_ - 2)

    if n < 0:
        print("Error: condition n >= 0 must be satisfied", file=sys.stderr)
    else:
        print(find_fib(n))
