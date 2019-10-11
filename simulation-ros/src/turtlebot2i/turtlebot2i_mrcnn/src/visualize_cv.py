import cv2
import numpy as np
import sys

class_names = ['BG', 'SlidingDoor', 'Wall', 'Shelf', 'Robot', 'Human', 'ConveyorBelt', 'Dockstation', 'Product', 'Floor']

def random_colors(N):
    np.random.seed(1)
    colors = [tuple(255 * np.random.rand(3)) for _ in range(N)]
    return colors

colors = random_colors(len(class_names))
class_dict = {
    name: color for name, color in zip(class_names, colors)
}

def apply_mask(image, mask, color, alpha=0.5):
    """apply mask to image"""
    for n, c in enumerate(color):
        image[:, :, n] = np.where(
            mask == 1,
            image[:, :, n] * (1 - alpha) + alpha * c,
            image[:, :, n]
        )
    return image

def display_instances(image, boxes, masks, ids, names, scores):
    """
        take the image and results and apply the mask, box, and Label
    """
    n_instances = boxes.shape[0]
    new_image = image.copy()

    if not n_instances:
        print('NO INSTANCES TO DISPLAY')
    else:
        #print (boxes.shape[0], masks.shape[-1], ids.shape[0])
        #assert boxes.shape[0] == masks.shape[-1] == ids.shape[0]
        assert boxes.shape[0] == ids.shape[0]

    for i in range(n_instances):
        if not np.any(boxes[i]):
            continue

        y1, x1, y2, x2 = boxes[i]
        label = names[ids[i]]

        print (i,' label: ',label)
        color = class_dict[class_names[ids[i]]]
        score = scores[i] if scores is not None else None
        caption = '{} {:.2f}'.format(label, score) if score else label
        if label != 'Floor':
            mask = masks[:, :, i]
            new_image = apply_mask(new_image, mask, color)
        #print (x1, y1,x2, y2)
        new_image = cv2.rectangle(new_image, (x1, y1), (x2, y2), color, 2)
        new_image = cv2.putText(
            new_image, caption, (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.7, color, 2
        )
    cv2.imshow('frame', new_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return new_image

def draw_bbox_label(image, boxes, ids, names, scores):
    """
        take the image and results and apply the box, and Label
    """
    n_instances = boxes.shape[0]
    new_image = image.copy()
    if not n_instances:
        print('NO INSTANCES TO DISPLAY')
    else:
        assert boxes.shape[0] == ids.shape[0]

    for i in range(n_instances):
        if not np.any(boxes[i]):
            continue

        y1, x1, y2, x2 = boxes[i]
        label = names[ids[i]]
        color = class_dict[label]
        score = scores[i] if scores is not None else None
        caption = '{} {:.2f}'.format(label, score) if score else label
        new_image = cv2.rectangle(new_image, (x1, y1), (x2, y2), color, 2)
        new_image = cv2.putText(
            new_image, caption, (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.7, color, 2
        )
    cv2.imshow('bbox label frame', new_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return new_image



def draw_bbox_label_msdn(image, boxes, ids, scores):
    """
        take the image and results and apply the box, and Label
    """
    n_instances = boxes.shape[0]
    new_image = image.copy()
    if not n_instances:
        print('NO INSTANCES TO DISPLAY')
    else:
        assert boxes.shape[0] == ids.shape[0]

    for i in range(n_instances):
        if not np.any(boxes[i]):
            continue

        x1, y1, x2, y2 = boxes[i]
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        label = class_names[ids[i]]
        color = class_dict[label]
        score = scores[i] if scores is not None else None
        caption = '{} {:.2f}'.format(label, score) if score else label
        new_image = cv2.rectangle(new_image, (x1, y1), (x2, y2), color, 2)
        new_image = cv2.putText(
            new_image, caption, (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.7, color, 2
        )
    cv2.imshow('bbox label frame', new_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return new_image

def draw_bbox(image, boxes):
    """
        take the image and bbox and apply the bbox
    """
    n_instances = boxes.shape[0]
    new_image = image.copy()

    if not n_instances:
        print('NO INSTANCES TO DISPLAY')
    else:
        assert boxes.shape[0] > 0

    for i in range(n_instances):
        if not np.any(boxes[i]):
            continue

        y1, x1, y2, x2 = boxes[i]
        color = class_dict["Robot"]
        caption = 'bbox'
        new_image = cv2.rectangle(new_image, (x1, y1), (x2, y2), color, 2)
        new_image = cv2.putText(
            new_image, caption, (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.7, color, 2
        )
    cv2.imshow('bbox frame', new_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return new_image
#if __name__ == '__main__':
#    """
#        test everything
#    """
#
#    capture = cv2.VideoCapture(0)
#
#    # these 2 lines can be removed if you dont have a 1080p camera.
#    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#
#    while True:
#        ret, frame = capture.read()
#        results = model.detect([frame], verbose=1)
#        r = results[0]
#        frame = display_instances(
#            frame, r['rois'], r['masks'], r['class_ids'], class_names, r['scores']
#        )
#        cv2.imshow('frame', frame)
#        if cv2.waitKey(1) & 0xFF == ord('q'):
#            break
#
#    capture.release()
#    cv2.destroyAllWindows()
