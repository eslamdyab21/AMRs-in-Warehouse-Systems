from QR import QR
import cv2

qr = QR()
# ---------------- Test 1 ------------------
qr.generator('[7,8]')

qr_img = cv2.imread("QR-[7,8].png")
decoded_msg = qr.reader(qr_img)

print(decoded_msg)


# ---------------- Test 2 ------------------
cap = cv2.VideoCapture('/dev/video2')
counter = 0
while True:
    ret, img = cap.read()
    if ret:
        decoded_msg = qr.reader(img)
        if decoded_msg:
            print(counter, ' decoded_msg: ', decoded_msg)
            print('---------------------------')

        cv2.imshow('scanner',img)
        if cv2.waitKey(1)==ord('q'):
            break
        counter = counter + 1