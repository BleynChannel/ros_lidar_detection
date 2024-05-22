import open3d as o3d
import numpy as np
from mmdet3d.apis import inference_detector
from mmdet3d.core.points.lidar_points import LiDARPoints

def rotate(source_angle, delta):
    result = source_angle + delta
    if result > np.pi:
        result = -np.pi + (result - np.pi)
    elif result < -np.pi:
        result = np.pi + (result + np.pi)
    return result

def get_slide_boxes(pointcloud_range, model_pcr_dim, apply_sw):
    pcd = pointcloud_range.copy()
    ws = model_pcr_dim.copy()
    if isinstance(apply_sw, dict):
        apply_sw = list(apply_sw.values())
    slides = [0,0,0]
    for i in range(3):
        if apply_sw is not None and not apply_sw[i]:
            slides[i] = 1
            continue
        slides[i], overlap = divmod(pcd[i], ws[i])
        if overlap > 0:
            slides[i] += 1

    sboxes = []
    for z in range(int(slides[2])):
        for y in range(int(slides[1])):
            for x in range(int(slides[0])):
                sboxes.append([
                    ws[0] * x,
                    ws[0] * (x + 1),
                    ws[1] * y,
                    ws[1] * (y + 1),
                    ws[2] * z,
                    ws[2] * (z + 1),
                ])
    return sboxes

def get_per_box_predictions(result, score_thr, selected_classes, cfg, center_vec, input_slide_range, gt_index_to_labels, model_name):
    if 'pts_bbox' in result[0].keys():
        preds = result[0]['pts_bbox']
    else:
        preds = result[0]

    pred_scores = preds['scores_3d'].numpy()
    pred_bboxes = preds['boxes_3d'].tensor.numpy() # x, y, z, dx, dy, dz, rot, vel_x, vel_y
    pred_labels = preds['labels_3d'].numpy()
    
    inds = pred_scores > score_thr
    pred_bboxes = pred_bboxes[inds]
    pred_labels = pred_labels[inds]
    pred_scores = pred_scores[inds]
    
    assert len(pred_bboxes) == len(pred_scores) == len(pred_labels)
    results = []
    for i in range(len(pred_bboxes)):
        det = {}
        det["detection_name"] = gt_index_to_labels[pred_labels[i]]
        if selected_classes is not None and det["detection_name"] not in selected_classes:
            continue
        det["size"] = pred_bboxes[i,3:6].tolist()
        if cfg.dataset_type != "SuperviselyDataset":
            det["size"] = [det["size"][1], det["size"][0], det["size"][2]]
        det["translation"] = pred_bboxes[i,:3].tolist()
        # TODO: check these conditions in the future
        if cfg.dataset_type != "SuperviselyDataset":
            det["translation"][2] += det["size"][2] * 0.5
        if model_name == "CenterPoint":
            det["translation"][2] += det["size"][2] * 0.5
            
        for k in range(3):
            det["translation"][k] += center_vec[k]
        # skip boxes out of pointcloud range
        if det["translation"][0] < input_slide_range[0] or \
            det["translation"][0] > input_slide_range[3] or \
            det["translation"][1] < input_slide_range[1] or \
            det["translation"][1] > input_slide_range[4] or \
            det["translation"][2] < input_slide_range[2] or \
            det["translation"][2] > input_slide_range[5]:
            continue
        det["rotation"] = pred_bboxes[i,6].item()
        if cfg.dataset_type != "SuperviselyDataset":
            det["rotation"] = rotate(det["rotation"], -np.pi * 0.5)
        det["velocity"] = pred_bboxes[i,7:].tolist()
        det["detection_score"] = pred_scores[i].item()
        results.append(det)
    return results

def inference_model(model, pcd, gt_index_to_labels, model_name, thresh=0.3, selected_classes=None,
                    apply_sw=None, center_ptc=None):
    if isinstance(center_ptc, dict):
        center_ptc = list(center_ptc.values())
    # check ptc ranges
    pcr = model.cfg.point_cloud_range
    pcr_dim = [pcr[3] - pcr[0], pcr[4] - pcr[1], pcr[5] - pcr[2]]
    input_ptc_dim = [
        pcd[:,0].max() - pcd[:,0].min(),
        pcd[:,1].max() - pcd[:,1].min(),
        pcd[:,2].max() - pcd[:,2].min()
    ]

    sboxes = get_slide_boxes(input_ptc_dim, pcr_dim, apply_sw)
    
    pcd_sboxes = []
    for sbox in sboxes:
        pcd_sbox = []
        for i in range(3):
            if center_ptc is None or center_ptc[i]:
                pcd_sbox.extend([
                    pcd[:,i].min() + sbox[i*2],
                    pcd[:,i].min() + sbox[i*2+1]
                ])
            else:
                pcd_sbox.extend([
                    pcr[i],
                    pcr[i+3]
                ])
        pcd_sboxes.append(pcd_sbox)

    results = []
    # TODO: is it possible to use batch inference here?
    for sbox in pcd_sboxes:
        pcd_eps = 1e-3
        pcd_slide = pcd[
            (pcd[:,0] > sbox[0] - pcd_eps) &
            (pcd[:,0] < sbox[1] + pcd_eps) &
            (pcd[:,1] > sbox[2] - pcd_eps) &
            (pcd[:,1] < sbox[3] + pcd_eps) &
            (pcd[:,2] > sbox[4] - pcd_eps) &
            (pcd[:,2] < sbox[5] + pcd_eps)
        ]
        if len(pcd_slide) == 0:
            continue
        center_vec = [0, 0, 0]
        input_slide_range = [
            pcd_slide[:,0].min(),
            pcd_slide[:,1].min(),
            pcd_slide[:,2].min(),
            pcd_slide[:,0].max(),
            pcd_slide[:,1].max(),
            pcd_slide[:,2].max()
        ]
        for i in range(3):
            if center_ptc is None or center_ptc[i]:
                dim_trans = input_slide_range[i] + (input_slide_range[i + 3] - input_slide_range[i]) * 0.5
                pcd_slide[:,i] -= dim_trans
                center_vec[i] = dim_trans

        intensity = np.zeros((pcd_slide.shape[0], 1), dtype=np.float32)
        pcd_slide = np.hstack((pcd_slide, intensity))
        points = LiDARPoints(pcd_slide, points_dim=pcd_slide.shape[-1])
        
        result, _ = inference_detector(model, points)
        result = get_per_box_predictions(result, thresh, selected_classes, model.cfg, center_vec, input_slide_range, gt_index_to_labels, model_name)
        results.extend(result)
    return results