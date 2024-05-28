point_cloud_range = [
    -7.478638800000001, -8.66540095, -3.170793775, 7.478638800000001,
    8.66540095, 3.170793775
]
class_names = ['people']
dataset_type = 'SuperviselyDataset'
data_root = '/app/data/sly_project'
input_modality = dict(
    use_lidar=True,
    use_camera=False,
    use_radar=False,
    use_map=False,
    use_external=False)
file_client_args = dict(backend='disk')
train_pipeline = [
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=4,
        use_dim=4,
        file_client_args=dict(backend='disk')),
    dict(type='LoadAnnotations3D', with_bbox_3d=True, with_label_3d=True),
    dict(
        type='GlobalRotScaleTrans',
        rot_range=[-0.3141592653589793, 0.3141592653589793],
        scale_ratio_range=[0.95, 1.05],
        translation_std=[0.5, 0.5, 0.5]),
    dict(
        type='RandomFlip3D',
        sync_2d=False,
        flip_ratio_bev_horizontal=0.2,
        flip_ratio_bev_vertical=0.2),
    dict(
        type='PointsRangeFilter',
        point_cloud_range=[
            -7.478638800000001, -8.66540095, -3.170793775, 7.478638800000001,
            8.66540095, 3.170793775
        ]),
    dict(
        type='ObjectRangeFilter',
        point_cloud_range=[
            -7.478638800000001, -8.66540095, -3.170793775, 7.478638800000001,
            8.66540095, 3.170793775
        ]),
    dict(type='ObjectNameFilter', classes=['people']),
    dict(type='PointShuffle'),
    dict(type='DefaultFormatBundle3D', class_names=['people']),
    dict(type='Collect3D', keys=['points', 'gt_bboxes_3d', 'gt_labels_3d'])
]
test_pipeline = [
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=4,
        use_dim=4,
        file_client_args=dict(backend='disk')),
    dict(
        type='MultiScaleFlipAug3D',
        img_scale=(1333, 800),
        pts_scale_ratio=1,
        flip=False,
        transforms=[
            dict(
                type='GlobalRotScaleTrans',
                rot_range=[0, 0],
                scale_ratio_range=[1.0, 1.0],
                translation_std=[0, 0, 0]),
            dict(type='RandomFlip3D'),
            dict(
                type='PointsRangeFilter',
                point_cloud_range=[
                    -7.478638800000001, -8.66540095, -3.170793775,
                    7.478638800000001, 8.66540095, 3.170793775
                ]),
            dict(
                type='DefaultFormatBundle3D',
                class_names=['people'],
                with_label=False),
            dict(type='Collect3D', keys=['points'])
        ])
]
eval_pipeline = [
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=4,
        use_dim=4,
        file_client_args=dict(backend='disk')),
    dict(
        type='DefaultFormatBundle3D', class_names=['people'],
        with_label=False),
    dict(type='Collect3D', keys=['points'])
]
data = dict(
    samples_per_gpu=4,
    workers_per_gpu=4,
    train=dict(
        pipeline=[
            dict(
                type='LoadPointsFromFile',
                coord_type='LIDAR',
                load_dim=4,
                use_dim=4,
                file_client_args=dict(backend='disk')),
            dict(
                type='LoadAnnotations3D',
                with_bbox_3d=True,
                with_label_3d=True),
            dict(
                type='GlobalRotScaleTrans',
                rot_range=[-0.3141592653589793, 0.3141592653589793],
                scale_ratio_range=[0.95, 1.05],
                translation_std=[0.5, 0.5, 0.5]),
            dict(
                type='RandomFlip3D',
                sync_2d=False,
                flip_ratio_bev_horizontal=0.2,
                flip_ratio_bev_vertical=0.2),
            dict(
                type='PointsRangeFilter',
                point_cloud_range=[
                    -7.478638800000001, -8.66540095, -3.170793775,
                    7.478638800000001, 8.66540095, 3.170793775
                ]),
            dict(
                type='ObjectRangeFilter',
                point_cloud_range=[
                    -7.478638800000001, -8.66540095, -3.170793775,
                    7.478638800000001, 8.66540095, 3.170793775
                ]),
            dict(type='ObjectNameFilter', classes=['people']),
            dict(type='PointShuffle'),
            dict(type='DefaultFormatBundle3D', class_names=['people']),
            dict(
                type='Collect3D',
                keys=['points', 'gt_bboxes_3d', 'gt_labels_3d'])
        ],
        data_root='/app/data/sly_project',
        type='SuperviselyDataset',
        ann_file='/app/data/train.pkl',
        test_mode=False,
        classes=['people'],
        filter_empty_gt=False),
    val=dict(
        pipeline=[
            dict(
                type='LoadPointsFromFile',
                coord_type='LIDAR',
                load_dim=4,
                use_dim=4,
                file_client_args=dict(backend='disk')),
            dict(
                type='MultiScaleFlipAug3D',
                img_scale=(1333, 800),
                pts_scale_ratio=1,
                flip=False,
                transforms=[
                    dict(
                        type='GlobalRotScaleTrans',
                        rot_range=[0, 0],
                        scale_ratio_range=[1.0, 1.0],
                        translation_std=[0, 0, 0]),
                    dict(type='RandomFlip3D'),
                    dict(
                        type='PointsRangeFilter',
                        point_cloud_range=[
                            -7.478638800000001, -8.66540095, -3.170793775,
                            7.478638800000001, 8.66540095, 3.170793775
                        ]),
                    dict(
                        type='DefaultFormatBundle3D',
                        class_names=['people'],
                        with_label=False),
                    dict(type='Collect3D', keys=['points'])
                ])
        ],
        data_root='/app/data/sly_project',
        type='SuperviselyDataset',
        ann_file='/app/data/val.pkl',
        test_mode=True,
        classes=['people']),
    test=dict(
        pipeline=[
            dict(
                type='LoadPointsFromFile',
                coord_type='LIDAR',
                load_dim=4,
                use_dim=4,
                file_client_args=dict(backend='disk')),
            dict(
                type='MultiScaleFlipAug3D',
                img_scale=(1333, 800),
                pts_scale_ratio=1,
                flip=False,
                transforms=[
                    dict(
                        type='GlobalRotScaleTrans',
                        rot_range=[0, 0],
                        scale_ratio_range=[1.0, 1.0],
                        translation_std=[0, 0, 0]),
                    dict(type='RandomFlip3D'),
                    dict(
                        type='PointsRangeFilter',
                        point_cloud_range=[
                            -7.478638800000001, -8.66540095, -3.170793775,
                            7.478638800000001, 8.66540095, 3.170793775
                        ]),
                    dict(
                        type='DefaultFormatBundle3D',
                        class_names=['people'],
                        with_label=False),
                    dict(type='Collect3D', keys=['points'])
                ])
        ],
        data_root='/app/data/sly_project',
        type='SuperviselyDataset',
        ann_file='/app/data/val.pkl',
        test_mode=True,
        classes=['people']),
    persistent_workers=True)
evaluation = dict(
    interval=1,
    pipeline=[
        dict(
            type='LoadPointsFromFile',
            coord_type='LIDAR',
            load_dim=4,
            use_dim=4,
            file_client_args=dict(backend='disk')),
        dict(
            type='DefaultFormatBundle3D',
            class_names=['people'],
            with_label=False),
        dict(type='Collect3D', keys=['points'])
    ],
    save_best='auto',
    rule='greater',
    out_dir='/app/data/artifacts/checkpoints',
    by_epoch=True)
voxel_size = [0.010386998333333335, 0.012035279097222223, 0.15853968875]
model = dict(
    type='CenterPointFixed',
    pts_voxel_layer=dict(
        max_num_points=10,
        voxel_size=[0.010386998333333335, 0.012035279097222223, 0.15853968875],
        max_voxels=(90000, 120000),
        point_cloud_range=[
            -7.478638800000001, -8.66540095, -3.170793775, 7.478638800000001,
            8.66540095, 3.170793775
        ]),
    pts_voxel_encoder=dict(type='HardSimpleVFE', num_features=4),
    pts_middle_encoder=dict(
        type='SparseEncoder',
        in_channels=4,
        sparse_shape=[41, 1440, 1440],
        output_channels=128,
        order=('conv', 'norm', 'act'),
        encoder_channels=((16, 16, 32), (32, 32, 64), (64, 64, 128), (128,
                                                                      128)),
        encoder_paddings=((0, 0, 1), (0, 0, 1), (0, 0, [0, 1, 1]), (0, 0)),
        block_type='basicblock'),
    pts_backbone=dict(
        type='SECOND',
        in_channels=256,
        out_channels=[128, 256],
        layer_nums=[5, 5],
        layer_strides=[1, 2],
        norm_cfg=dict(type='BN', eps=0.001, momentum=0.01),
        conv_cfg=dict(type='Conv2d', bias=False)),
    pts_neck=dict(
        type='SECONDFPN',
        in_channels=[128, 256],
        out_channels=[256, 256],
        upsample_strides=[1, 2],
        norm_cfg=dict(type='BN', eps=0.001, momentum=0.01),
        upsample_cfg=dict(type='deconv', bias=False),
        use_conv_for_no_stride=True),
    pts_bbox_head=dict(
        type='CenterHeadWithVel',
        in_channels=512,
        tasks=[dict(num_class=1, class_names=['people'])],
        common_heads=dict(reg=(2, 2), height=(1, 2), dim=(3, 2), rot=(2, 2)),
        share_conv_channel=64,
        bbox_coder=dict(
            type='CenterPointBBoxCoder',
            post_center_range=[
                -7.478638800000001, -8.66540095, -3.170793775,
                7.478638800000001, 8.66540095, 3.170793775
            ],
            max_num=500,
            score_threshold=0.1,
            out_size_factor=8,
            voxel_size=[0.010386998333333335, 0.012035279097222223],
            code_size=7,
            pc_range=[
                -7.478638800000001, -8.66540095, -3.170793775,
                7.478638800000001, 8.66540095, 3.170793775
            ]),
        separate_head=dict(
            type='DCNSeparateHead',
            init_bias=-2.19,
            final_kernel=3,
            dcn_config=dict(
                type='DCN',
                in_channels=64,
                out_channels=64,
                kernel_size=3,
                padding=1,
                groups=4)),
        loss_cls=dict(type='GaussianFocalLoss', reduction='mean'),
        loss_bbox=dict(type='L1Loss', reduction='mean', loss_weight=0.25),
        norm_bbox=True),
    train_cfg=dict(
        pts=dict(
            grid_size=[1440, 1440, 40],
            voxel_size=[
                0.010386998333333335, 0.012035279097222223, 0.15853968875
            ],
            out_size_factor=8,
            dense_reg=1,
            gaussian_overlap=0.1,
            max_objs=500,
            min_radius=2,
            code_weights=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            point_cloud_range=[
                -7.478638800000001, -8.66540095, -3.170793775,
                7.478638800000001, 8.66540095, 3.170793775
            ])),
    test_cfg=dict(
        pts=dict(
            post_center_limit_range=[
                -7.478638800000001, -8.66540095, -3.170793775,
                7.478638800000001, 8.66540095, 3.170793775
            ],
            max_per_img=500,
            max_pool_nms=False,
            min_radius=[4, 12, 10, 1, 0.85, 0.175],
            score_threshold=0.1,
            out_size_factor=8,
            voxel_size=[0.010386998333333335, 0.012035279097222223],
            nms_type='circle',
            pre_max_size=1000,
            post_max_size=83,
            nms_thr=0.2,
            pc_range=[
                -7.478638800000001, -8.66540095, -3.170793775,
                7.478638800000001, 8.66540095, 3.170793775
            ])))
optimizer = dict(
    type='Adam', lr=0.001, weight_decay=0, betas=(0.9, 0.999), amsgrad=False)
optimizer_config = dict(grad_clip=dict(max_norm=30, norm_type=2))
lr_config = dict(
    policy='Step',
    by_epoch=True,
    warmup=None,
    warmup_by_epoch=False,
    warmup_iters=0,
    warmup_ratio=0.1,
    step=4,
    gamma=0.1,
    min_lr=0)
momentum_config = dict(
    policy='cyclic',
    target_ratio=(0.8947368421052632, 1),
    cyclic_times=1,
    step_ratio_up=0.4)
runner = dict(type='EpochBasedRunner', max_epochs=20)
checkpoint_config = dict(
    interval=1,
    by_epoch=True,
    max_keep_ckpts=3,
    out_dir='/app/data/artifacts/checkpoints')
log_config = dict(
    interval=10, hooks=[dict(type='SuperviselyLoggerHook', by_epoch=False)])
dist_params = dict(backend='nccl')
log_level = 'INFO'
work_dir = '/app/data'
load_from = None
resume_from = None
workflow = [('train', 1)]
opencv_num_threads = 0
mp_start_method = 'fork'
db_sampler = None
pretrained_model = 'CenterPoint'
center_coords = [True, True, True]
seed = 0
gpu_ids = range(0, 1)
