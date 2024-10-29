import cv2
import time

cap = cv2.VideoCapture(0)
intervals = []

n = 200
start_all = time.time()
for _ in range(n):
    start = time.time()
    ret, frame = cap.read()
    end = time.time()
    intervals.append(end-start)
    start = end
end_all = time.time()

print(n/(end_all-start_all))
print(min(intervals)**(-1))
print(max(intervals)**(-1))
print((sum(intervals)/len(intervals))**(-1))
# print(ret, frame)