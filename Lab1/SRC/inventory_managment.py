# Product class to store product details and manage stock
class Product:
    def __init__(self, name, price, stock):
        # Initialize product with name, price, and initial stock
        self.name = name
        self.price = price
        self.stock = stock

    def sell(self, quantity):
        # Reduce stock by given quantity if available
        if self.stock >= quantity:
            self.stock -= quantity
            return True
        return False

    def get_info(self):
        # Return formatted product information string
        return f"Product: {self.name} | Price: ${self.price:.2f} | Stock: {self.stock} units"

    @property
    def total_value(self):
        # Calculate total value of current stock
        return self.price * self.stock

# Inventory class to manage collection of products
class Inventory:
    def __init__(self):
        # Initialize empty product list
        self.products = []

    def add_product(self, product):
        # Add product to inventory
        self.products.append(product)

    def sell_product(self, product_name, quantity):
        # Find product by name and attempt sale
        for product in self.products:
            if product.name == product_name:
                return product.sell(quantity)
        return False

    def display_product_info(self, product_name):
        # Display information for specified product
        for product in self.products:
            if product.name == product_name:
                print(product.get_info())
                return
        print("Product not found!")

    def calculate_total_value(self):
        # Calculate total value of all inventory
        return sum(product.total_value for product in self.products)

# Main program execution-------------------------------------------------------------------------before this is heavaen awesome, after is hell, the idea of making interphases make my skin crawl 
if __name__ == "__main__":
    inventory = Inventory()
    
    while True:
        # Display menu
        print("\nInventory Management System")
        print("1. Add Product")
        print("2. Sell Product")
        print("3. View Product Info")
        print("4. Calculate Total Inventory Value")
        print("5. Exit")
        
        # Get user choice
        try:
            choice = int(input("Enter choice: "))
        except ValueError:
            print("Invalid input. Please enter a number.")
            continue
        
        # Handle menu options
        if choice == 1:
            # Add new product
            name = input("Enter product name: ").strip()
            
            # Get valid price
            while True:
                try:
                    price = float(input("Enter product price: "))
                    if price < 0:
                        print("Price cannot be negative")
                        continue
                    break
                except ValueError:
                    print("Invalid price format")
            
            # Get valid stock
            while True:
                try:
                    stock = int(input("Enter initial stock: "))
                    if stock < 0:
                        print("Stock cannot be negative")
                        continue
                    break
                except ValueError:
                    print("Invalid stock format")
            
            inventory.add_product(Product(name, price, stock))
            print("Product added successfully!")
        
        elif choice == 2:
            # Sell products
            product_name = input("Enter product name: ").strip()
            
            # Get valid quantity
            while True:
                try:
                    quantity = int(input("Enter quantity to sell: "))
                    if quantity < 0:
                        print("Quantity cannot be negative")
                        continue
                    break
                except ValueError:
                    print("Invalid quantity format")
            
            if inventory.sell_product(product_name, quantity):
                print("Sale successful!")
            else:
                print("Sale failed (product not found or insufficient stock)")
        
        elif choice == 3:
            # View product info
            product_name = input("Enter product name: ").strip()
            inventory.display_product_info(product_name)
        
        elif choice == 4:
            # Calculate total value
            total = inventory.calculate_total_value()
            print(f"Total inventory value: ${total:.2f}")
        
        elif choice == 5:
            # Exit program
            print("Exiting...")
            break
        
        else:
            print("Invalid choice. Please enter 1-5.")
