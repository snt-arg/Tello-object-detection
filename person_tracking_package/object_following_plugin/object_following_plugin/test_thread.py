
import time
from threading import Thread

def test_function(a1:int, a2:float):
    print("\n#################\n#################\nExecuting the thread\n#################\n#################\n")
    time.sleep(0.005)
    print("Result : ",a1+a2)
    print("\n#################\n#################\nDone with the thread\n#################\n#################\n")
    
thread1 = Thread(target=test_function,args=(2,3),daemon=True)



counter = 0

while(True):
    print("normal flow : ", counter)
    if counter==20:
        thread1.start()
    time.sleep(0.001)
        #thread1.run()
    counter += 1
    if counter == 100:
        break
thread1.start()
    