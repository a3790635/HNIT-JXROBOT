def isConfirm():
    print(" [Y]es/[N]o: ")
    yes = {"Y", "y", "Yes", "yes", "YES"}
    no = {"N", "n", "No", "no", "NO"}
    inputStr = input()
    while True:
        if inputStr in yes:
            return True
        if inputStr in no:
            return False

        print("Please enter again.")
