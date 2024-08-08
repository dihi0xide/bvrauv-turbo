# import cv2 as cv
#
# cam = cv.VideoCapture(0)
#
# if not cam.isOpened():
#     print("Cannot open camera")
#     exit(1)
#
# while True:
#     ret, frame = cam.read()
#
#     # ret is whether the frame was correctly captured
#     if not ret:
#         print("Can't receive frame (maybe end of video)")
#         break
#
#     hsv_img = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
#
#     red_mask = cv.inRange(hsv_img, (0,50,20), (15,255,255)) + cv.inRange(hsv_img, (170,50,20), (180,255,255))
#
#
# # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#     cv.imshow('frame', cv.bitwise_and(hsv_img, red_mask))
#
#     if cv.waitKey(1) == ord('q'):
#         break
#
# cam.release()
# cv.destroyAllWindows()