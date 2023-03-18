from QR import QR
import cv2

qr = QR()

qr.generator('[7,8]')

qr_img = cv2.imread("QR-[7,8].png")
decoded_msg = qr.reader(qr_img)

print(decoded_msg)