import statistics
import math

numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

# Get even numbers 
even_nums = [num for num in numbers if num % 2 == 0] 
avg = statistics.mean(even_nums)
print(f"Even numbers average: {avg}")  

# Get ODD numbers (not divisible by 2)
odd_nums = [num for num in numbers if num % 2 != 0]  
prod = math.prod(odd_nums)
print(f"Product of odd numbers: {prod}")  
