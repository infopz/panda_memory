import torch
from torch import optim, nn
from torchvision import models, transforms
from numpy.linalg import norm
import numpy as np
import cv2


class FeatureExtractor(nn.Module):
    def __init__(self, model):
        super(FeatureExtractor, self).__init__()
        # Extract VGG-16 Feature Layers
        self.features = list(model.features)
        self.features = nn.Sequential(*self.features)
        # Extract VGG-16 Average Pooling Layer
        self.pooling = model.avgpool
        # Convert the image into one-dimensional vector
        self.flatten = nn.Flatten()
        # Extract the first part of fully-connected layer from VGG16
        self.fc = model.classifier[0]

    def forward(self, x):
        # It will take the input 'x' until it returns the feature vector called 'out'
        out = self.features(x)
        out = self.pooling(out)
        out = self.flatten(out)
        out = self.fc(out)
        return out


# Initialize the model
model = models.vgg16(pretrained=True)
new_model = FeatureExtractor(model)

# Change the device to GPU
device = torch.device('cuda:0' if torch.cuda.is_available() else "cpu")
new_model = new_model.to(device)

# Transform the image, so it becomes readable with the model
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.CenterCrop(512),
    transforms.Resize(448),
    transforms.ToTensor()
])


def vgg_descriptor(img):
    vgg_des = []
    img = cv2.convertScaleAbs(img, 0.5, 2.5)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img = img[105:230, 65:190]
    img = transform(img)
    img = img.reshape(1, 3, 448, 448)
    img = img.to(device)
    with torch.no_grad():
        feature = new_model(img)
    vgg_des.append(feature.cpu().detach().numpy().reshape(-1))
    vgg_des = np.array(vgg_des)

    return vgg_des


def vgg_compare(des1, des2):
    # Cosine similarity

    cosine = np.sum(des1 * des2, axis=1) / (norm(des1, axis=1) * norm(des2, axis=1))

    return cosine > 0.9
