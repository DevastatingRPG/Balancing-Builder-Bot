def calculate_value(t1, t2):
    part1 = ((60 - t1) / 60) * 20
    part2 = ((100 - (t2 - t1)) / 100) * 30
    return part1 + part2

# Example: Let's compute with t1 = 10 and t2 = 50 as test values
print(calculate_value(1.2400000000000009, 3.3399999999999728))