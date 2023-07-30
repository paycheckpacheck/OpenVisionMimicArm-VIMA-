import threading


import time

#my blocking fucntion
def count(n):

    for i in range(1, n+1):
        print(f'{threading.current_thread().name}: {i}')
        time.sleep(1)
def main():
    start_time = time.time()

    #Define 3 working threads
    t1 = threading.Thread(target=count, args=(5,), name='Thread 1')
    t2 = threading.Thread(target=count, args=(5,), name='Thread 2')
    t3 = threading.Thread(target=count, args=(5,), name='Thread 3')


    #Make each thread start executing each respective blocking fucntion
    t1.start()
    t2.start()
    t3.start()
    #Join each thread frio synchronous execution
    t1.join()
    t2.join()
    t3.join()
    end_time = time.time()


    total_time = abs(start_time - end_time)

    return total_time

if __name__ == '__main__':
    threadingTime = main()


    print("Time: ", threadingTime)

