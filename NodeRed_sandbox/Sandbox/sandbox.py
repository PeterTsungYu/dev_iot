# Python program showing a use of
# global in nested function
x = 10
def add():
    #global x
    #x = 15
       
    def change():
        global x
        x = 20
    print("Before making changing: ", x)
    print("Making change")
    change()
    print("After making change: ", x)
  
add()
print("value of x", x)