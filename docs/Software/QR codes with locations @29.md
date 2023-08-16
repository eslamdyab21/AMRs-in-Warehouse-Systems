>This is a documentation to the issue #29, the `QR.py` is the script developed to tackle this issue.

</br>

First we start by importing 2 libraries which we will make use of.
```python
import qrcode
import cv2
```

- `qrcode` for the qr code generation
- `cv2` for decoding the qr code

Then we create the class with two main methods `generator` function generates/encodes a qr image with a desired message, and `reader` function reads/decodes a QR-image.
```python
import qrcode
import cv2

class QR():

	def __init__(self):
		pass


	def generator(self, msg, path_to_save_qr_img = None):
	
		# Generate QR code
		qr_img = qrcode.make(msg)
		qr_img.save(f"QR-{msg}.png")
		
		# Resize QR code image
		qr_img = cv2.imread(f"QR-{msg}.png")
		width = qr_img.shape[0]
		height = qr_img.shape[1]
		dim = (int(width*0.5), int(0.5*height))
		qr_img = cv2.resize(qr_img, dim)
		
		# Save QR code image
		if path_to_save_qr_img == None:
			cv2.imwrite(f"QR-{msg}.png", qr_img)
		else:
			cv2.imwrite(f"{path_to_save_qr_img}/QR-{msg}.png", qr_img)


	def reader(self, qr_img):
		detector=cv2.QRCodeDetector()
		decoded_msg , one, _=detector.detectAndDecode(qr_img)
		
		return decoded_msg
```

</br>
</br>

#### Test case
```python
from QR import QR
import cv2

qr = QR()

qr.generator('[7,8]')

qr_img = cv2.imread("QR-[7,8].png")
decoded_msg = qr.reader(qr_img)  

print(decoded_msg)
```

- generated qr image:

![](images/QR-[7,8].png)

- decoded message
```python  
❯ python3 main.py 
[7,8]
```



</br>
</br>
</br>


#### Real camera test case
```python
from QR import QR
import cv2

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
```

- Camera is placed at height of `8, 10 and 14 cm`
- The QR code size is `5.5x5.5 cm2` 

![](images/qr_camera_test.mp4)

https://user-images.githubusercontent.com/77991372/220340311-cf8227c8-c322-4c68-af61-c96a5493f1da.mp4

