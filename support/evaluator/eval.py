import numpy as np
from PIL import Image
import yaml

def image_rmse(img1, img2, normalize=False):
    """
    Calculate RMSE between two images pixel by pixel, ignoring pixels that are transparent in both images
    
    Args:
        img1: First image (numpy array or PIL Image)
        img2: Second image (numpy array or PIL Image)
        normalize: Whether to normalize images to 0-1 range
    
    Returns:
        RMSE value (float)
    """
    # Convert to numpy arrays if needed
    if isinstance(img1, Image.Image):
        img1 = np.array(img1)
    if isinstance(img2, Image.Image):
        img2 = np.array(img2)
    
    # Ensure same shape
    assert img1.shape == img2.shape, "Images must have same dimensions"
    
    # Convert to float to avoid overflow
    img1 = img1.astype(np.float64)
    img2 = img2.astype(np.float64)
    
    # Handle transparency (alpha channel)
    if img1.shape[-1] == 4:  # RGBA
        # Get alpha channels
        alpha1 = img1[..., 3]
        alpha2 = img2[..., 3]
        
        # Create mask: pixels where at least one image is non-transparent
        # (alpha > 0 means non-transparent)
        mask = (alpha1 > 0) | (alpha2 > 0)
        
        # Only use RGB channels for comparison
        img1_rgb = img1[..., :3]
        img2_rgb = img2[..., :3]
        
        # Apply mask to get only relevant pixels
        if np.sum(mask) == 0:
            # No non-transparent pixels found
            return 0.0
            
        img1_masked = img1_rgb[mask]
        img2_masked = img2_rgb[mask]
        
    else:  # RGB or Grayscale
        # No alpha channel, use all pixels
        img1_masked = img1.flatten()
        img2_masked = img2.flatten()
    
    # Normalize to 0-1 if requested
    if normalize:
        img1_masked = img1_masked / 255.0
        img2_masked = img2_masked / 255.0
    
    # Calculate RMSE on masked pixels only
    mse = np.mean((img1_masked - img2_masked) ** 2)
    return np.sqrt(mse)

def evaluate_images_from_yaml(yaml_path, normalize=False):
    """
    Load images from YAML config and calculate RMSE
    
    Args:
        yaml_path: Path to YAML configuration file
        normalize: Whether to normalize images to 0-1 range
    
    Returns:
        RMSE value (float)
    """
    # Load YAML config
    with open(yaml_path, 'r') as file:
        config = yaml.safe_load(file)
    
    # Get image paths
    painted_path = config['painted_uri']
    observed_path = config['observed_uri']
    
    # Load images
    painted_img = Image.open(painted_path)
    observed_img = Image.open(observed_path)
    
    # Calculate RMSE
    rmse_value = image_rmse(painted_img, observed_img, normalize=normalize)
    
    print(f"Painted image: {painted_path}")
    print(f"Observed image: {observed_path}")
    print(f"Normalized: {normalize}")
    print(f"RMSE: {rmse_value:.6f}")
    
    return rmse_value
    
def interpret_rmse(rmse_value, normalized=False):
    """
    Interpret RMSE value quality
    
    Args:
        rmse_value: RMSE result
        normalized: Whether images were normalized (0-1) or not (0-255)
    
    Returns:
        Quality interpretation string
    """
    if not normalized:  # 8-bit images (0-255)
        if rmse_value == 0:
            return "Really? - Absolutely identical"
        elif rmse_value < 5:
            return "Excellent - Almost identical"
        elif rmse_value < 15:
            return "Good - Minor differences"
        elif rmse_value < 30:
            return "Fair - Noticeable differences"
        elif rmse_value < 50:
            return "Poor - Significant differences"
        else:
            return "Very Poor - Major differences"
    else:  # Normalized images (0-1)
        if rmse_value == 0:
            return "Really? - Absolutely identical"
        elif rmse_value < 0.02:
            return "Excellent - Almost identical"
        elif rmse_value < 0.06:
            return "Good - Minor differences"
        elif rmse_value < 0.12:
            return "Fair - Noticeable differences"
        elif rmse_value < 0.20:
            return "Poor - Significant differences"
        else:
            return "Very Poor - Major differences"

if __name__ == "__main__":
    # Evaluate images from YAML config
    yaml_path = "/home/lamp/workspaces/segnet/support/evaluator/eval.yaml"
    
    # Test both normalized and non-normalized
    print("=== 8-bit Images (0-255) ===")
    rmse_result = evaluate_images_from_yaml(yaml_path, normalize=False)
    interpretation = interpret_rmse(rmse_result, normalized=False)
    print(f"Quality Assessment: {interpretation}")
    
    print("\n=== Normalized Images (0-1) ===")
    rmse_result_norm = evaluate_images_from_yaml(yaml_path, normalize=True)
    interpretation_norm = interpret_rmse(rmse_result_norm, normalized=True)
    print(f"Quality Assessment: {interpretation_norm}")