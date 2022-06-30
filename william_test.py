# Calculator

# Get numbers from user
num1 = float(input("Enter first number: "))
num2 = float(input("Enter second number: "))

# Get input operation from user
operation = input("Enter an operation to perform (+, -, * or /): ")

case = {
    "+": num1 + num2,
    "-": num1 - num2,
    "*": num1 * num2,
    "/": num1 / num2
}

# if operation input is not in the set [case], print error
if operation not in case:
    print("Error: Invalid operation")
else:
    print(case[operation])

input("Press enter to exit")