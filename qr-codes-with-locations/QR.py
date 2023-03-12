import qrcode
import cv2


class QR():
    """
    QR class genrates and read qr code with custom messages
    """
    def __init__(self):
        self.detector = cv2.QRCodeDetector()
        pass

    def generator(self, msg, path_to_save_qr_img = None):
        """
        generate function generates/encodes a qr image with a desired message

        :param msg: (string) message to be encoded in a qr image
        :param path_to_save_qr_img: (string) (optional) path in which the 
                                    qr image will be stored, if None is provieded
                                    the image will be saved in current relative path
        """
        
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
        """
        reader function reads/decodes a QR-image

        :param qr_img: (opencv-image) QR-image to be decoded
        :return decoded_msg: (string) with decoded data
        """

        decoded_msg = None
        
        try:
            decoded_msg , one, _= self.detector.detectAndDecode(qr_img)
        except:
            print('error in qr decoding')

        return decoded_msg


