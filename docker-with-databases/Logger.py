# importing module
import logging


class Logger():
    """
    Logger class to log debugging and info data
    """
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)

        formatter = logging.Formatter('%(asctime)s: %(name)s: %(message)s')

        file_handler = logging.FileHandler('logger.log')
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)

        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)

        self.logger.addHandler(file_handler)
        # self.logger.addHandler(stream_handler)
    

    def log(self, msg):
        """
        log messages
        :param msg: message to be logged
        """

        """
        logger.debug("Harmless debug Message")
        logger.info("Just an information")
        logger.warning("Its a Warning")
        logger.error("Did you try to divide by zero")
        logger.critical("Internet is down")
        """
        # self.logger.debug(msg)
        self.logger.info(msg)
