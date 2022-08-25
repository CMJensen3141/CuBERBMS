from tabulate import tabulate
 
# assign data
Holding_Registers = [
    ["Current 1", 0, 1, 2],
    ["Current 2", 1, 1, 2],
    ["Current 3", 2, 1, 2],
    ["Power Reference", 8, 1, 2]
]
 
# create header
head = ["Name", "Address", "Count", "Unit"]
 
# display table
print(tabulate(Holding_Registers, headers=head, tablefmt="grid"))

# print(Holding_Registers[0])