import multiprocessing
import time
def count(n):
    for i in range(1, n+1):
        print(f'{multiprocessing.current_process().name}: {i}')
        time.sleep(1)

def main():

    start_time = time.time()
    with multiprocessing.Pool(processes=3) as pool:
        pool.map(count, [5, 5, 5])

    end_time = time.time()

    total_Time = abs(start_time - end_time)
    return total_Time

if __name__ == '__main__':


    mp_time  = main()

    print("Time: ", mp_time)
