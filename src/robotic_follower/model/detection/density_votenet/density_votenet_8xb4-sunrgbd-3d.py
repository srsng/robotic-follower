auto_scale_lr = dict(base_batch_size=128, enable=True)
backend_args = None
class_names = (
    'bed',
    'table',
    'sofa',
    'chair',
    'toilet',
    'desk',
    'dresser',
    'night_stand',
    'bookshelf',
    'bathtub',
)
data_root = 'data2/sunrgbd/'
dataset_type = 'SUNRGBDDataset'
default_hooks = dict(
    checkpoint=dict(interval=1, type='CheckpointHook'),
    logger=dict(interval=50, type='LoggerHook'),
    param_scheduler=dict(type='ParamSchedulerHook'),
    sampler_seed=dict(type='DistSamplerSeedHook'),
    timer=dict(type='IterTimerHook'),
    visualization=dict(type='Det3DVisualizationHook'))
default_scope = 'mmdet3d'
env_cfg = dict(
    cudnn_benchmark=False,
    dist_cfg=dict(backend='nccl'),
    mp_cfg=dict(mp_start_method='fork', opencv_num_threads=0))
launcher = 'none'
load_from = None
log_level = 'INFO'
log_processor = dict(by_epoch=True, type='LogProcessor', window_size=50)
lr = 0.008
metainfo = dict(
    classes=(
        'bed',
        'table',
        'sofa',
        'chair',
        'toilet',
        'desk',
        'dresser',
        'night_stand',
        'bookshelf',
        'bathtub',
    ))
model = dict(
    backbone=dict(
        fp_channels=(
            (
                256,
                256,
            ),
            (
                256,
                256,
            ),
        ),
        in_channels=5,
        norm_cfg=dict(type='BN2d'),
        num_points=(
            2048,
            1024,
            512,
            256,
        ),
        num_samples=(
            64,
            32,
            16,
            16,
        ),
        radius=(
            0.2,
            0.4,
            0.8,
            1.2,
        ),
        sa_cfg=dict(
            normalize_xyz=True,
            pool_mod='max',
            type='PointSAModule',
            use_xyz=True),
        sa_channels=(
            (
                64,
                64,
                128,
            ),
            (
                128,
                128,
                256,
            ),
            (
                128,
                128,
                256,
            ),
            (
                128,
                128,
                256,
            ),
        ),
        type='DensityAwarePointNet2'),
    bbox_head=dict(
        bbox_coder=dict(
            mean_sizes=[
                [
                    2.114,
                    2.393,
                    0.855,
                ],
                [
                    0.605,
                    0.699,
                    0.712,
                ],
                [
                    2.644,
                    1.056,
                    0.916,
                ],
                [
                    0.601,
                    0.601,
                    0.942,
                ],
                [
                    0.425,
                    0.437,
                    0.761,
                ],
                [
                    0.414,
                    1.165,
                    0.738,
                ],
                [
                    1.055,
                    0.595,
                    1.022,
                ],
                [
                    0.496,
                    0.509,
                    0.782,
                ],
                [
                    0.341,
                    2.151,
                    1.836,
                ],
                [
                    1.321,
                    0.681,
                    0.582,
                ],
            ],
            num_dir_bins=12,
            num_sizes=10,
            type='PartialBinBasedBBoxCoder',
            with_rot=True),
        center_loss=dict(
            loss_dst_weight=10.0,
            loss_src_weight=10.0,
            mode='l2',
            reduction='sum',
            type='ChamferDistance'),
        dir_class_loss=dict(
            loss_weight=1.0, reduction='sum', type='mmdet.CrossEntropyLoss'),
        dir_res_loss=dict(
            loss_weight=10.0, reduction='sum', type='mmdet.SmoothL1Loss'),
        num_classes=10,
        objectness_loss=dict(
            class_weight=[
                0.2,
                0.8,
            ],
            loss_weight=5.0,
            reduction='sum',
            type='mmdet.CrossEntropyLoss'),
        pred_layer_cfg=dict(
            bias=True, in_channels=128, shared_conv_channels=(
                128,
                128,
            )),
        semantic_loss=dict(
            loss_weight=1.0, reduction='sum', type='mmdet.CrossEntropyLoss'),
        size_class_loss=dict(
            loss_weight=1.0, reduction='sum', type='mmdet.CrossEntropyLoss'),
        size_res_loss=dict(
            loss_weight=3.3333333333333335,
            reduction='sum',
            type='mmdet.SmoothL1Loss'),
        type='VoteHead',
        vote_aggregation_cfg=dict(
            mlp_channels=[
                256,
                128,
                128,
                128,
            ],
            normalize_xyz=True,
            num_point=256,
            num_sample=16,
            radius=0.3,
            type='PointSAModule',
            use_xyz=True),
        vote_module_cfg=dict(
            conv_cfg=dict(type='Conv1d'),
            conv_channels=(
                256,
                256,
            ),
            gt_per_seed=3,
            in_channels=256,
            norm_cfg=dict(type='BN1d'),
            norm_feats=True,
            vote_loss=dict(
                loss_dst_weight=10.0,
                mode='l1',
                reduction='none',
                type='ChamferDistance'),
            vote_per_seed=1)),
    data_preprocessor=dict(type='Det3DDataPreprocessor'),
    neck=dict(
        groups=4,
        in_channels=256,
        num_blocks=1,
        reduction=4,
        type='CGNLLocalFusionNeck'),
    test_cfg=dict(
        nms_thr=0.25,
        per_class_proposal=True,
        sample_mode='seed',
        score_thr=0.05),
    train_cfg=dict(
        neg_distance_thr=0.6, pos_distance_thr=0.3, sample_mode='vote'),
    type='VoteNet')
optim_wrapper = dict(
    clip_grad=dict(max_norm=10, norm_type=2),
    optimizer=dict(lr=0.008, type='AdamW', weight_decay=0.01),
    type='OptimWrapper')
param_scheduler = [
    dict(
        begin=0,
        by_epoch=True,
        end=36,
        gamma=0.1,
        milestones=[
            24,
            32,
        ],
        type='MultiStepLR'),
]
resume = False
test_cfg = dict(type='TestLoop')
test_dataloader = dict(
    batch_size=20,
    dataset=dict(
        ann_file='sunrgbd_infos_val.pkl',
        backend_args=None,
        box_type_3d='Depth',
        data_root='data2/sunrgbd/',
        metainfo=dict(
            classes=(
                'bed',
                'table',
                'sofa',
                'chair',
                'toilet',
                'desk',
                'dresser',
                'night_stand',
                'bookshelf',
                'bathtub',
            )),
        pipeline=[
            dict(
                backend_args=None,
                coord_type='DEPTH',
                load_dim=6,
                shift_height=True,
                type='LoadPointsFromFile',
                use_dim=[
                    0,
                    1,
                    2,
                ]),
            dict(
                flip=False,
                img_scale=(
                    1333,
                    800,
                ),
                pts_scale_ratio=1,
                transforms=[
                    dict(
                        rot_range=[
                            0,
                            0,
                        ],
                        scale_ratio_range=[
                            1.0,
                            1.0,
                        ],
                        translation_std=[
                            0,
                            0,
                            0,
                        ],
                        type='GlobalRotScaleTrans'),
                    dict(
                        flip_ratio_bev_horizontal=0.5,
                        sync_2d=False,
                        type='RandomFlip3D'),
                    dict(num_points=20000, type='PointSample'),
                    dict(k_neighbor=64, sigma=0.1, type='ComputePointDensity'),
                ],
                type='MultiScaleFlipAug3D'),
            dict(keys=[
                'points',
            ], type='Pack3DDetInputs'),
        ],
        test_mode=True,
        type='SUNRGBDDataset'),
    num_workers=1,
    sampler=dict(shuffle=False, type='DefaultSampler'))
test_evaluator = dict(type='IndoorMetric')
test_pipeline = [
    dict(
        backend_args=None,
        coord_type='DEPTH',
        load_dim=6,
        shift_height=True,
        type='LoadPointsFromFile',
        use_dim=[
            0,
            1,
            2,
        ]),
    dict(
        flip=False,
        img_scale=(
            1333,
            800,
        ),
        pts_scale_ratio=1,
        transforms=[
            dict(
                rot_range=[
                    0,
                    0,
                ],
                scale_ratio_range=[
                    1.0,
                    1.0,
                ],
                translation_std=[
                    0,
                    0,
                    0,
                ],
                type='GlobalRotScaleTrans'),
            dict(
                flip_ratio_bev_horizontal=0.5,
                sync_2d=False,
                type='RandomFlip3D'),
            dict(num_points=20000, type='PointSample'),
            dict(k_neighbor=64, sigma=0.1, type='ComputePointDensity'),
        ],
        type='MultiScaleFlipAug3D'),
    dict(keys=[
        'points',
    ], type='Pack3DDetInputs'),
]
train_cfg = dict(max_epochs=36, type='EpochBasedTrainLoop', val_interval=1)
train_dataloader = dict(
    batch_size=20,
    dataset=dict(
        dataset=dict(
            ann_file='sunrgbd_infos_train.pkl',
            backend_args=None,
            box_type_3d='Depth',
            data_root='data2/sunrgbd/',
            filter_empty_gt=False,
            metainfo=dict(
                classes=(
                    'bed',
                    'table',
                    'sofa',
                    'chair',
                    'toilet',
                    'desk',
                    'dresser',
                    'night_stand',
                    'bookshelf',
                    'bathtub',
                )),
            pipeline=[
                dict(
                    backend_args=None,
                    coord_type='DEPTH',
                    load_dim=6,
                    shift_height=True,
                    type='LoadPointsFromFile',
                    use_dim=[
                        0,
                        1,
                        2,
                    ]),
                dict(type='LoadAnnotations3D'),
                dict(
                    flip_ratio_bev_horizontal=0.5,
                    sync_2d=False,
                    type='RandomFlip3D'),
                dict(
                    rot_range=[
                        -0.523599,
                        0.523599,
                    ],
                    scale_ratio_range=[
                        0.85,
                        1.15,
                    ],
                    shift_height=True,
                    type='GlobalRotScaleTrans'),
                dict(num_points=20000, type='PointSample'),
                dict(k_neighbor=64, sigma=0.1, type='ComputePointDensity'),
                dict(
                    keys=[
                        'points',
                        'gt_bboxes_3d',
                        'gt_labels_3d',
                    ],
                    type='Pack3DDetInputs'),
            ],
            type='SUNRGBDDataset'),
        times=5,
        type='RepeatDataset'),
    num_workers=4,
    sampler=dict(shuffle=True, type='DefaultSampler'))
train_pipeline = [
    dict(
        backend_args=None,
        coord_type='DEPTH',
        load_dim=6,
        shift_height=True,
        type='LoadPointsFromFile',
        use_dim=[
            0,
            1,
            2,
        ]),
    dict(type='LoadAnnotations3D'),
    dict(flip_ratio_bev_horizontal=0.5, sync_2d=False, type='RandomFlip3D'),
    dict(
        rot_range=[
            -0.523599,
            0.523599,
        ],
        scale_ratio_range=[
            0.85,
            1.15,
        ],
        shift_height=True,
        type='GlobalRotScaleTrans'),
    dict(num_points=20000, type='PointSample'),
    dict(k_neighbor=64, sigma=0.1, type='ComputePointDensity'),
    dict(
        keys=[
            'points',
            'gt_bboxes_3d',
            'gt_labels_3d',
        ],
        type='Pack3DDetInputs'),
]
val_cfg = dict(type='ValLoop')
val_dataloader = dict(
    batch_size=20,
    dataset=dict(
        ann_file='sunrgbd_infos_val.pkl',
        backend_args=None,
        box_type_3d='Depth',
        data_root='data2/sunrgbd/',
        metainfo=dict(
            classes=(
                'bed',
                'table',
                'sofa',
                'chair',
                'toilet',
                'desk',
                'dresser',
                'night_stand',
                'bookshelf',
                'bathtub',
            )),
        pipeline=[
            dict(
                backend_args=None,
                coord_type='DEPTH',
                load_dim=6,
                shift_height=True,
                type='LoadPointsFromFile',
                use_dim=[
                    0,
                    1,
                    2,
                ]),
            dict(
                flip=False,
                img_scale=(
                    1333,
                    800,
                ),
                pts_scale_ratio=1,
                transforms=[
                    dict(
                        rot_range=[
                            0,
                            0,
                        ],
                        scale_ratio_range=[
                            1.0,
                            1.0,
                        ],
                        translation_std=[
                            0,
                            0,
                            0,
                        ],
                        type='GlobalRotScaleTrans'),
                    dict(
                        flip_ratio_bev_horizontal=0.5,
                        sync_2d=False,
                        type='RandomFlip3D'),
                    dict(num_points=20000, type='PointSample'),
                    dict(k_neighbor=64, sigma=0.1, type='ComputePointDensity'),
                ],
                type='MultiScaleFlipAug3D'),
            dict(keys=[
                'points',
            ], type='Pack3DDetInputs'),
        ],
        test_mode=True,
        type='SUNRGBDDataset'),
    num_workers=1,
    sampler=dict(shuffle=False, type='DefaultSampler'))
val_evaluator = dict(type='IndoorMetric')
vis_backends = [
    dict(type='LocalVisBackend'),
    dict(type='TensorboardVisBackend'),
]
visualizer = dict(
    name='visualizer',
    type='Det3DLocalVisualizer',
    vis_backends=[
        dict(type='LocalVisBackend'),
        dict(type='TensorboardVisBackend'),
    ])
work_dir = 'work_dirs/density_votenet_8xb4-sunrgbd-3d'
