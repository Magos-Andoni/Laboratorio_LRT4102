# Dictionary storing employee data with nested dictionaries for hourly wage and hours
employees = {
    "Sara": {
        "hourly wage": 27,  # Sara's hourly rate in dollars
        "hours": 4          # Number of hours Sara worked
    },
    "Cesar":{
      "costo por hora": 27,
      "horas": 1
    },
    "Malcm":{
      "costo por hora": 27,
      "horas": 8
  },
    "Andoni":{
      "costo por hora": 27,
      "horas": 5
  },
    "Nacho":{
      "costo por hora": 27,
      "horas": 8
  },
    "Johnny":{
      "costo por hora": 27,
      "horas": 9
  },
}

# Function to calculate and display salaries for all employees
def salary():
    # Iterate through each employee in the dictionary
    for name, data in employees.items():
        # Calculate salary: hourly wage × hours worked
        sal = data["hourly wage"] * data["hours"]
        # Display formatted result with employee name and calculated salary
        print(f"The salary of {name} is: $ {sal} ")

# Execute the salary calculation function
salary()
