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
        generat function generates/encodes a qr image with a desired message

        :param msg: (string) message to be encoded in a qr image
        :param path_to_save_qr_img: (string) (optional) path in which the 
                                    qr image will be stored, if None is provieded
                                    the image will be saved in current relative path
        """

        qr_img = qrcode.make(msg)

        if path_to_save_qr_img == None:
            qr_img.save(f"QR-{msg}.png")
        else:
            qr_img.save(f"{path_to_save_qr_img}/QR-{msg}.png")



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


