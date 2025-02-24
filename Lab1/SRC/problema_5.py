# Import random module for generating the secret number
import random

def guess_game():
    # Generate secret number between 1-10 and initialize game variables
    secret_number = random.randint(1, 10)  # The holy grail number
    attempts = 0       # Track how many tries the player needs
    got_it = False     # Flag to check if they've guessed right

    # Drop some streetwise instructions
    print("Ight man this one ain't easy....")
    print("Guess the number I'm thinking of, it's between 1 and 10")

    # Main game loop - keeps running until they guess right
    while not got_it:
        try:
            # Get player's guess and convert to integer
            guess = int(input("Say a number, lil' man..."))
            attempts += 1  # Increase try counter

            # Check if guess needs to go up or down
            if guess < secret_number:
                print("Oof! Too low, lil' gangster. Try again!")
            elif guess > secret_number:
                print("Too high, brother! Go lower.")
            else:
                # Correct guess - end the game
                got_it = True
                print(f"Bull's eye, brother! It only took you {attempts} tries... dayum.")
                
        except ValueError:
            # Handle non-number inputs like a streetwise bouncer
            print("No no, don't try to hustle me. That ain't an integer...")

# Start the game by calling the function
guess_game()
