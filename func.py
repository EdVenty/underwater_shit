from typing import List
import numpy as np
import cv2

def white_balance(img: np.ndarray) -> np.ndarray:
    # авто коррекция баланса белого
    """Corrects white balance of image.

    Args:
        img (np.ndarray): Cv2 frame

    Returns:
        np.ndarray: Corrected cv2 frame
    """
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result

def adjust_gamma(image: np.ndarray, invgamma: float=1.0) -> np.ndarray:
    # подстройка гаммы изображения
    """Adjust gamme of image with specified gamme value.

    Args:
        image (np.ndarray): Cv2 frame
        invgamma (float, optional): Inverted gamma. Gamma = 1 / imvgamma. Defaults to 1.0. About gamma: https://www.cambridgeincolour.com/ru/tutorials-ru/gamma-correction.htm

    Returns:
        np.ndarray: Corrected cv2 frame
    """
    invGamma = 1.0 / invgamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")

    return cv2.LUT(image, table)

def gaussian_blur(image: np.ndarray, sigma: float) -> np.ndarray:
    # блюр по гаусу
    return cv2.GaussianBlur(image, (0, 0), sigma)

def unsharp_mask(image: np.ndarray, alpha: float=1.5, beta: float=-0.5, gamma: float=0, gaussian_sigma: float=2.0) -> np.ndarray:
    # выделение "острых" участтков изображения
    """Unsharp mask filter.

    Args:
        image (np.ndarray): Cv2 image
        alpha (float, optional): Weight of first image. Defaults to 1.5.
        beta (float, optional): Weight of secong image. Defaults to -0.5.
        gamma (float, optional): Scalar added to each them. Defaults to 0.
        gaussian_sigma (float, optional): [description]. Defaults to 2.0.

    Returns:
        np.ndarray: Processed cv2 image
    """
    gaussian = gaussian_blur(image, gaussian_sigma)
    return cv2.addWeighted(image, alpha, gaussian, beta, gamma, image)

def CLAHE(image: np.ndarray, clipLimit=3.0, tileGridSize=(8,8)) -> np.ndarray:
    # алгоритм CLAHE для коррекции освещения изображения
    """CLAHE filter for cv2 image.

    Args:
        image (np.ndarray): Cv2 BGR image.
        clipLimit (float, optional): Threshold for contrast limiting. Defaults to 3.0.
        tileGridSize (tuple, optional): Size of grid for histogram equalization. Defaults to (8,8).

    Returns:
        np.ndarray: Processed cv2 BGR image.
    """
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl,a,b))
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    return final

def laplace(image: np.ndarray, ddepth: float=cv2.CV_16S, kernel_size: float=3) -> np.ndarray:
    # Выделение областей картинки по глубине алгоритмом Лапласа
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dst = cv2.Laplacian(gray, ddepth, ksize=kernel_size)
    abs_dst = cv2.convertScaleAbs(dst)
    return abs_dst

def silency(image: np.ndarray) -> np.ndarray:
    # Выделение бросающихся глазу человека участков изображения
    saliency = cv2.saliency.StaticSaliencyFineGrained_create()
    (success, saliencyMap) = saliency.computeSaliency(image)
    return saliencyMap.astype("uint8")

def fusion(images: List[np.ndarray]) -> np.ndarray:
    mergeMertens = cv2.createMergeMertens()
    exposureFusion = mergeMertens.process(images)
    return exposureFusion
