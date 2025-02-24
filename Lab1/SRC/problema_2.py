# Get number of hours worked from user input
horas = int(input("Aight man, tell me how many hours did you punch in today? "))

# Get hourly wage from user input (in dollars)
money = int(input("Bet man, now tell me how much gold ya making out per hour... "))

# Calculate total earnings (hours multiplied by hourly rate)
salario = horas * money

# Display the result showing total earnings
print("Those suits owe ya ", salario, "dollars man")
