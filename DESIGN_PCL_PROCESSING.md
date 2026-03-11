# 点云处理功能包设计文档

## 概述

本文档设计一个统一的点云处理功能包，整合现有的点云处理功能，并提供标准化的接口。

## 项目信息

- **包名**: `pcl_processing`
- **位置**: `/home/srsnn/ros2_ws/src/pcl_processing`
- **ROS 版本**: ROS2 Humble
- **编程语言**: C++ (支持 Python 接口)
- **依赖**: PCL 1.x, OpenCV, Eigen3, ROS2 sensor_msgs

## 设计目标

1. **统一接口**: 提供标准化、易用的点云处理 API
2. **模块化设计**: 功能模块独立，可按需组合使用
3. **兼容性**: 兼容多种深度相机（RealSense D435、Kinect2 等）
4. **高性能**: 支持点云并行处理和 GPU 加速（可选）
5. **可扩展**: 便于添加新的点云处理算法

## 功能模块设计

### 1. 深度图像转点云模块 (`DepthToPointCloud`)

**功能**:
- 将深度图像 + 相机内参转换为 3D 点云
- 支持 16 位无符号整数深度图像
- 支持浮点深度图像
- 可配置深度缩放因子和无效深度值处理

**接口**:
```cpp
class DepthToPointCloud {
public:
    // 构造函数
    DepthToPointCloud(const CameraIntrinsics& intrinsics);

    // 深度图转点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr convert(
        const cv::Mat& depth_image,
        float depth_scale = 1.0f,
        float invalid_depth_value = 0.0f
    );

    // 带颜色的深度图转点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convert(
        const cv::Mat& depth_image,
        const cv::Mat& rgb_image,
        float depth_scale = 1.0f,
        float invalid_depth_value = 0.0f
    );

    // 从 ROS 消息转换
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertFromROS(
        const sensor_msgs::msg::Image::SharedPtr depth_msg,
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info
    );
};
```

**实现要点**:
- 使用相机内参矩阵将像素坐标 (u, v) 和深度 z 转换为 3D 坐标
- 公式: X = (u - cx) * z / fx, Y = (v - cy) * z / fy
- 处理深度图像的缩放因子（RealSense D435 默认 0.001）

### 2. 点云滤波模块

#### 2.1 体素滤波 (`VoxelFilter`)

**功能**:
- 下采样点云，减少数据量
- 使用体素网格合并相邻点
- 可配置体素大小

**接口**:
```cpp
class VoxelFilter {
public:
    VoxelFilter(float leaf_size = 0.01f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );

    void setLeafSize(float x, float y, float z);
};
```

**实现要点**:
- 使用 PCL 的 `pcl::VoxelGrid<pcl::PointXYZ>`
- 默认体素大小 1cm

#### 2.2 统计滤波 (`StatisticalFilter`)

**功能**:
- 移除离群点（噪声点）
- 基于点云统计特性（距离邻域点的平均距离）
- 可配置统计窗口大小和标准差倍数

**接口**:
```cpp
class StatisticalFilter {
public:
    StatisticalFilter(int mean_k = 50, float std_dev = 1.0f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );

    void setParameters(int mean_k, float std_dev);
};
```

**实现要点**:
- 使用 PCL 的 `pcl::StatisticalOutlierRemoval<pcl::PointXYZ>`
- 默认考虑每个点的 50 个邻居
- 默认标准差倍数为 1.0

#### 2.3 空间滤波 (`PassThroughFilter`)

**功能**:
- 在指定轴方向上截取点云范围
- 支持多轴连续滤波

**接口**:
```cpp
class PassThroughFilter {
public:
    PassThroughFilter();

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );

    void setFilterLimits(const std::string& field_name, float min, float max);
    void setFilterLimitsX(float min, float max);
    void setFilterLimitsY(float min, float max);
    void setFilterLimitsZ(float min, float max);
};
```

#### 2.4 半径滤波 (`RadiusFilter`)

**功能**:
- 移除半径内邻居数量不足的点
- 用于去除稀疏噪声

**接口**:
```cpp
class RadiusFilter {
public:
    RadiusFilter(float radius = 0.05f, int min_neighbors = 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );
};
```

### 3. 点云分割模块

#### 3.1 平面分割 (`PlaneSegmentation`)

**功能**:
- 使用 RANSAC 检测平面（如地面、桌面）
- 返回平面系数和内点索引
- 支持检测多个平面

**接口**:
```cpp
struct PlaneResult {
    pcl::ModelCoefficients coefficients;  // [a, b, c, d] where ax + by + cz + d = 0
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers;
    float plane_height;  // 平面高度（z 坐标）
};

class PlaneSegmentation {
public:
    PlaneSegmentation(float distance_threshold = 0.05f);

    PlaneResult segment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );

    std::vector<PlaneResult> segmentMultiPlane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int max_planes = 3
    );
};
```

#### 3.2 欧式聚类分割 (`EuclideanClustering`)

**功能**:
- 根据欧式距离将点云分割为多个聚类
- 返回每个聚类的点云和中心点
- 常用于物体检测

**接口**:
```cpp
struct ClusterResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
    pcl::PointXYZ centroid;
    int point_count;
};

class EuclideanClustering {
public:
    EuclideanClustering(float cluster_tolerance = 0.1f,
                       int min_cluster_size = 100,
                       int max_cluster_size = 25000);

    std::vector<ClusterResult> cluster(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );
};
```

### 4. 点云坐标变换模块 (`PointCloudTransformer`)

**功能**:
- TF 坐标变换
- 刚体变换
- 支持从 ROS 消息自动变换

**接口**:
```cpp
class PointCloudTransformer {
public:
    PointCloudTransformer(rclcpp::Node::SharedPtr node);

    // TF 变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& target_frame,
        const std::string& source_frame,
        const rclcpp::Time& time
    );

    // 从 ROS PointCloud2 消息转换到目标坐标系
    sensor_msgs::msg::PointCloud2 transformROS(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg,
        const std::string& target_frame
    );

    // 应用刚体变换矩阵
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyTransform(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Matrix4f& transform
    );
};
```

### 5. 点云可视化模块 (`PointCloudVisualizer`)

**功能**:
- 使用 RViz 可视化点云
- 发布点云话题
- 支持发布标记（包围盒、平面等）

**接口**:
```cpp
class PointCloudVisualizer {
public:
    PointCloudVisualizer(rclcpp::Node::SharedPtr node,
                         const std::string& topic_name = "/processed_pointcloud");

    void publish(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    void publishColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    void publishMarkers(const std::vector<pcl::PointXYZ>& points,
                        const std::string& marker_type = "sphere");
};
```

### 6. 点云查找表模块 (`PointCloudLUT`)

**功能**:
- 生成 2D 图像坐标到 3D 点云索引的查找表
- 快速查询图像像素对应的 3D 点
- 支持二进制和文本格式存储

**接口**:
```cpp
class PointCloudLUT {
public:
    // 从点云和相机参数生成查找表
    void generate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                  const CameraIntrinsics& intrinsics);

    // 查询 2D 像素对应的 3D 点
    bool queryPoint(int u, int v, pcl::PointXYZ& point) const;

    // 保存/加载查找表
    void save(const std::string& filename, bool binary = true);
    void load(const std::string& filename, bool binary = true);
};
```

### 7. 综合处理管道 (`PointCloudPipeline`)

**功能**:
- 组合多个处理步骤
- 支持动态配置
- 一键处理原始点云

**接口**:
```cpp
class PointCloudPipeline {
public:
    PointCloudPipeline();

    // 添加处理步骤
    void addFilter(std::shared_ptr<PointCloudFilter> filter);
    void addSegmentation(std::shared_ptr<PointCloudSegmentation> segmentation);

    // 执行处理管道
    pcl::PointCloud<pcl::PointXYZ>::Ptr process(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input
    );

    // 获取中间结果
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getIntermediateResults();
};
```

### 8. 密度计算模块 (`DensityCalculator`)

**功能**:
- 使用核密度估计（KDE）计算每个点的局部密度
- 支持高斯核和均匀核
- 离线计算密度，用于深度学习网络
- 可配置带宽和归一化方式

**接口**:
```cpp
class DensityCalculator {
public:
    enum class KernelType {
        GAUSSIAN,
        UNIFORM,
        EPANECHNIKOV
    };

    enum class NormalizationType {
        MINMAX,      // 归一化到 [0, 1]
        ZSCORE,      // 标准化（均值为0，标准差为1）
        NONE         // 不归一化
    };

    DensityCalculator(KernelType kernel = KernelType::GAUSSIAN,
                     float bandwidth = 0.5f);

    // 计算点云密度
    std::vector<float> computeDensity(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );

    // 使用 KD 树加速计算（基于 k 近邻）
    std::vector<float> computeDensityFast(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int k_neighbors = 50
    );

    // 归一化密度
    void normalizeDensity(std::vector<float>& density,
                         NormalizationType norm_type = NormalizationType::MINMAX);

    // 保存/加载密度
    void save(const std::string& filename, const std::vector<float>& density);
    std::vector<float> load(const std::string& filename);

    void setBandwidth(float bandwidth);
    void setKernelType(KernelType kernel);
};
```

**实现要点**:
- 使用 KD 树加速邻域搜索
- 高斯核公式: f(x) = (1/(nh)) * Σ K((x-xi)/h)
- 支持并行计算提高效率
- 密度归一化到合理范围（如 [0, 1]）

### 9. 3D 目标检测网络模块 (`ObjectDetection3D`)

**功能**:
- 基于密度信息与局部特征融合的 3D 目标检测
- 输入点云 + 密度信息，输出 3D 边界框
- 支持训练和推理模式
- 使用 PyTorch 作为后端

**接口** (Python):
```python
from pcl_processing.detection import ObjectDetection3D, ModelConfig

# 模型配置
config = ModelConfig(
    num_classes=18,              # 类别数量
    input_points=10000,         # 输入点数
    feature_dim=256,            # 特征维度
    num_proposals=256,          # 提议数量
    use_density_fusion=True,     # 是否使用密度融合
    use_cgnl=True               # 是否使用 CGNL 模块
)

# 创建检测器
detector = ObjectDetection3D(config)

# 加载预训练模型
detector.load_model('path/to/model.pth')

# 推理
detections = detector.detect(point_cloud, density)

# 输出格式
for det in detections:
    print(f"Class: {det.class_name}, "
          f"Confidence: {det.confidence:.2f}, "
          f"Center: {det.center}, "
          f"Size: {det.size}, "
          f"Heading: {det.heading:.2f}")
```

**网络结构**:
1. **输入层**: 点云 (N×3) + 密度 (N×1)
2. **下采样层（4层）**: FPS + 密度融合 + MLP + 最大池化
3. **上采样层（2层）**: 特征传播
4. **CGNL 模块**: 局部特征融合
5. **投票网络**: VoteNet 风格的投票层
6. **Proposal 模块**: 回归 3D 边界框参数

### 10. 网络子模块

#### 10.1 基于密度的下采样层 (`DensitySetAbstraction`)

**接口** (Python):
```python
class DensitySetAbstraction(nn.Module):
    def __init__(self, npoint, radius, nsample, in_channel, mlp, use_density=True):
        """
        Args:
            npoint: 采样点数量
            radius: 分组半径
            nsample: 每组采样点数
            in_channel: 输入特征维度
            mlp: MLP 各层输出维度列表
            use_density: 是否使用密度融合
        """
        super().__init__()
        self.npoint = npoint
        self.radius = radius
        self.nsample = nsample
        self.use_density = use_density

        # MLP1: 点特征升维
        self.mlp_convs = nn.ModuleList([...])
        self.mlp_bns = nn.ModuleList([...])

        # MLP2: 密度变换（如果启用）
        if use_density:
            self.density_mlp = nn.Sequential(
                nn.Conv1d(1, mlp[-1], 1),
                nn.BatchNorm1d(mlp[-1]),
                nn.ReLU()
            )

    def forward(self, xyz, points, density):
        """
        Args:
            xyz: 点坐标 (B, N, 3)
            points: 点特征 (B, C, N) 或 None
            density: 密度信息 (B, 1, N)

        Returns:
            new_xyz: 下采样后的中心点坐标 (B, npoint, 3)
            new_points: 下采样后的特征 (B, feature_dim, npoint)
        """
        # 1. FPS 采样中心点
        fps_idx = farthest_point_sample(xyz, self.npoint)
        new_xyz = index_points(xyz, fps_idx)

        # 2. 球查询分组
        idx = query_ball_point(self.radius, self.nsample, xyz, new_xyz)
        grouped_xyz = index_points(xyz, idx)
        grouped_xyz_norm = grouped_xyz - new_xyz.unsqueeze(2)

        # 3. 特征拼接
        if points is not None:
            grouped_points = torch.cat([grouped_xyz_norm, grouped_points], dim=-1)
        else:
            grouped_points = grouped_xyz_norm

        # 4. MLP1 特征提取
        grouped_points = grouped_points.permute(0, 3, 1, 2)
        for conv, bn in zip(self.mlp_convs, self.mlp_bns):
            grouped_points = F.relu(bn(conv(grouped_points)))

        # 5. 密度融合
        if self.use_density:
            grouped_density = index_points(density.transpose(1, 2), fps_idx)
            density_weight = self.density_mlp(grouped_density)
            density_weight = density_weight.unsqueeze(-1)
            grouped_points = grouped_points * density_weight

        # 6. 最大池化
        new_points = torch.max(grouped_points, -1)[0]

        return new_xyz, new_points
```

#### 10.2 CGNL 模块 (`CompactGeneralizedNonLocal`)

**功能**:
- 紧凑型广义非局部网络
- 通过分组减少计算量
- 建立局部特征间的关联

**接口** (Python):
```python
class CompactGeneralizedNonLocal(nn.Module):
    def __init__(self, in_channels, groups=4):
        """
        Args:
            in_channels: 输入特征维度
            groups: 分组数量（减少计算量）
        """
        super().__init__()
        self.groups = groups
        self.channels_per_group = in_channels // groups

        # θ, φ, g 变换
        self.theta = nn.Conv1d(in_channels, in_channels, 1)
        self.phi = nn.Conv1d(in_channels, in_channels, 1)
        self.g = nn.Conv1d(in_channels, in_channels, 1)

        # 输出变换
        self.out_conv = nn.Conv1d(in_channels, in_channels, 1)
        self.bn = nn.BatchNorm1d(in_channels)

    def forward(self, x):
        """
        Args:
            x: 输入特征 (B, C, M)，M 为点数

        Returns:
            out: 输出特征 (B, C, M)
        """
        B, C, M = x.shape

        # 计算变换
        theta = self.theta(x)  # (B, C, M)
        phi = self.phi(x)      # (B, C, M)
        g = self.g(x)          # (B, C, M)

        # 分组
        theta = theta.view(B, self.groups, self.channels_per_group, M)
        phi = phi.view(B, self.groups, self.channels_per_group, M)
        g = g.view(B, self.groups, self.channels_per_group, M)

        # 计算相似性矩阵（每组内）
        sim = torch.einsum('bgcm,bgcn->bgmn', theta, phi)
        sim = F.softmax(sim / (self.channels_per_group ** 0.5), dim=-1)

        # 加权求和
        out = torch.einsum('bgmn,bgcn->bgcm', sim, g)
        out = out.contiguous().view(B, C, M)

        # 残差连接
        out = self.out_conv(out)
        out = self.bn(out)
        out = F.relu(out + x)

        return out
```

#### 10.3 投票网络模块 (`VoteNetModule`)

**功能**:
- VoteNet 风格的投票机制
- 每个点预测偏移到物体中心
- 采样和聚类生成 proposals

**接口** (Python):
```python
class VoteNetModule(nn.Module):
    def __init__(self, num_classes, num_points, num_proposals):
        """
        Args:
            num_classes: 类别数量
            num_points: 输入点数
            num_proposals: 提议数量
        """
        super().__init__()
        self.num_classes = num_classes
        self.num_proposals = num_proposals

        # 投票层：预测偏移和目标性
        self.vote_mlp = nn.Sequential([
            nn.Conv1d(256, 256, 1),
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.Conv1d(256, 256, 1),
            nn.BatchNorm1d(256),
            nn.ReLU()
        ])
        self.vote_layer = nn.Conv1d(256, 3 + 1 + num_classes, 1)

        # Proposal 层：回归 3D 框
        self.proposal_mlp = nn.Sequential([
            nn.Conv1d(256, 128, 1),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.Conv1d(128, 128, 1),
            nn.BatchNorm1d(128),
            nn.ReLU()
        ])
        self.proposal_layer = nn.Conv1d(128, 3 + 3 + 1 + num_classes, 1)

    def forward(self, xyz, features):
        """
        Args:
            xyz: 点坐标 (B, N, 3)
            features: 点特征 (B, C, N)

        Returns:
            proposals: 检测结果列表
        """
        # 投票预测
        vote_features = self.vote_mlp(features)
        vote_output = self.vote_layer(vote_features)
        # vote_output: (B, 3+1+C, N) -> [dx, dy, dz, objectness, class_scores]

        # 提取投票信息
        vote_xyz = xyz + vote_output[:, :3, :].transpose(1, 2)
        objectness = vote_output[:, 3, :]
        class_scores = vote_output[:, 4:, :]

        # 采样投票点
        sampled_idx = torch.topk(objectness, self.num_proposals, dim=1)[1]
        sampled_xyz = index_points(vote_xyz, sampled_idx)
        sampled_features = index_points(features.transpose(1, 2), sampled_idx)

        # Proposal 回归
        proposal_features = self.proposal_mlp(sampled_features.transpose(1, 2))
        proposal_output = self.proposal_layer(proposal_features)
        # proposal_output: [center_offset, size, heading, class_scores]

        return self.parse_proposals(sampled_xyz, proposal_output)
```

### 11. 训练与评估模块

#### 11.1 训练器 (`DetectionTrainer`)

**接口** (Python):
```python
class DetectionTrainer:
    def __init__(self, model, train_loader, val_loader, config):
        """
        Args:
            model: ObjectDetection3D 模型
            train_loader: 训练数据加载器
            val_loader: 验证数据加载器
            config: 训练配置
        """
        self.model = model
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.config = config

        # 优化器
        self.optimizer = torch.optim.Adam(
            model.parameters(),
            lr=config.learning_rate
        )

        # 学习率调度器
        self.scheduler = torch.optim.lr_scheduler.StepLR(
            self.optimizer,
            step_size=60,
            gamma=0.1
        )

        # 损失函数
        self.criterion = DetectionLoss()

    def train(self, num_epochs):
        """训练主循环"""
        for epoch in range(num_epochs):
            # 训练一个 epoch
            train_loss = self.train_epoch(epoch)

            # 验证
            val_loss, val_map = self.validate(epoch)

            # 保存模型
            if (epoch + 1) % 20 == 0:
                self.save_model(epoch)

    def train_epoch(self, epoch):
        """训练一个 epoch"""
        self.model.train()
        total_loss = 0

        for batch_idx, (points, density, labels) in enumerate(self.train_loader):
            # 前向传播
            predictions = self.model(points, density)

            # 计算损失
            loss = self.criterion(predictions, labels)

            # 反向传播
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            total_loss += loss.item()

        return total_loss / len(self.train_loader)

    def validate(self, epoch):
        """验证"""
        self.model.eval()
        # ... 验证逻辑
        pass
```

#### 11.2 损失函数 (`DetectionLoss`)

**接口** (Python):
```python
class DetectionLoss(nn.Module):
    def __init__(self, config):
        """
        总损失包括：
        - 投票损失（偏移回归）
        - 目标性损失（二分类）
        - 分类损失（多分类）
        - 框回归损失（中心、尺寸、角度）
        """
        super().__init__()
        self.config = config

        # 损失权重
        self.vote_loss_weight = 1.0
        self.objectness_loss_weight = 5.0
        self.class_loss_weight = 1.0
        self.box_loss_weight = 1.0

    def forward(self, predictions, labels):
        """
        计算总损失
        """
        # 投票损失
        vote_loss = self.compute_vote_loss(predictions, labels)

        # 目标性损失
        objectness_loss = self.compute_objectness_loss(predictions, labels)

        # 分类损失
        class_loss = self.compute_class_loss(predictions, labels)

        # 框回归损失
        box_loss = self.compute_box_loss(predictions, labels)

        # 总损失
        total_loss = (
            self.vote_loss_weight * vote_loss +
            self.objectness_loss_weight * objectness_loss +
            self.class_loss_weight * class_loss +
            self.box_loss_weight * box_loss
        )

        return total_loss
```

#### 11.3 评估器 (`DetectionEvaluator`)

**接口** (Python):
```python
class DetectionEvaluator:
    def __init__(self, num_classes, iou_thresholds=[0.25, 0.5]):
        """
        Args:
            num_classes: 类别数量
            iou_thresholds: IoU 阈值列表（用于计算 mAP）
        """
        self.num_classes = num_classes
        self.iou_thresholds = iou_thresholds

    def evaluate(self, predictions, gt_labels):
        """
        评估检测结果

        Returns:
            dict: 包含 mAP@0.25, mAP@0.5 等指标
        """
        # 计算每个 IoU 阈值下的 mAP
        results = {}
        for iou_th in self.iou_thresholds:
            aps = self.compute_average_precision(predictions, gt_labels, iou_th)
            map_ = np.mean(aps)
            results[f'mAP@{iou_th}'] = map_

        return results

    def compute_iou(self, box1, box2):
        """
        计算 3D 边界框的 IoU

        Args:
            box1: [center_x, center_y, center_z, size_x, size_y, size_z, heading]
            box2: 同上

        Returns:
            float: IoU 值
        """
        # 旋转后的 IoU 计算逻辑
        # ...
        pass
```

### 12. 数据加载模块

#### 12.1 点云数据集 (`PointCloudDataset`)

**接口** (Python):
```python
class PointCloudDataset(torch.utils.data.Dataset):
    def __init__(self, data_root, split='train', num_points=10000):
        """
        Args:
            data_root: 数据集根目录
            split: 'train' 或 'val'
            num_points: 每个场景采样点数
        """
        self.data_root = data_root
        self.split = split
        self.num_points = num_points

        # 加载数据索引
        self.samples = self.load_samples()

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        """
        返回：
            points: (N, 3) 点云坐标
            density: (N,) 密度信息
            labels: 检测框标签
        """
        sample = self.samples[idx]

        # 加载点云
        points = self.load_points(sample['point_file'])

        # 加载密度（如果存在）
        density = self.load_density(sample['density_file'])
        if density is None:
            # 实时计算密度
            density = compute_density(points)

        # 加载标签
        labels = self.load_labels(sample['label_file'])

        # 数据增强
        if self.split == 'train':
            points, labels = self.augment(points, labels)

        # 下采样
        points, density = self.subsample(points, density, self.num_points)

        return points, density, labels

    def augment(self, points, labels):
        """数据增强：随机旋转、平移、抖动"""
        # 随机旋转
        angle = np.random.uniform(-np.pi, np.pi)
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        rotation = np.array([
            [cos_angle, -sin_angle, 0],
            [sin_angle, cos_angle, 0],
            [0, 0, 1]
        ])
        points = points @ rotation.T

        # 随机平移
        translation = np.random.uniform(-0.1, 0.1, 3)
        points = points + translation

        # 随机抖动
        jitter = np.random.normal(0, 0.01, points.shape)
        points = points + jitter

        return points, labels
```

### 13. 可视化模块

#### 13.1 检测结果可视化 (`DetectionVisualizer`)

**接口** (Python):
```python
class DetectionVisualizer:
    def __init__(self, use_open3d=True):
        """
        Args:
            use_open3d: 是否使用 Open3D 可视化
        """
        self.use_open3d = use_open3d

    def visualize(self, points, detections):
        """
        可视化点云和检测结果

        Args:
            points: (N, 3) 点云坐标
            detections: 检测结果列表
        """
        if self.use_open3d:
            self.visualize_open3d(points, detections)
        else:
            self.visualize_matplotlib(points, detections)

    def visualize_open3d(self, points, detections):
        """使用 Open3D 可视化"""
        import open3d as o3d

        # 创建点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color([0.7, 0.7, 0.7])

        # 创建边界框
        bboxes = []
        colors = self.get_class_colors()
        for det in detections:
            center = det.center
            size = det.size
            heading = det.heading

            # 创建 3D 边界框
            bbox = self.create_3d_bbox(center, size, heading, colors[det.class_id])
            bboxes.append(bbox)

        # 渲染
        o3d.visualization.draw_geometries([pcd] + bboxes)

    def save_visualization(self, points, detections, output_path):
        """保存可视化结果为图像"""
        # ... 保存逻辑
        pass
```

### 14. 模型导出与部署

#### 14.1 ONNX 导出 (`ModelExporter`)

**接口** (Python):
```python
class ModelExporter:
    def __init__(self, model):
        self.model = model

    def export_to_onnx(self, output_path, input_shape=(1, 3, 10000)):
        """
        导出模型到 ONNX 格式

        Args:
            output_path: ONNX 文件输出路径
            input_shape: 输入张量形状
        """
        import torch.onnx

        # 创建示例输入
        dummy_points = torch.randn(*input_shape)
        dummy_density = torch.randn(1, 1, 10000)

        # 导出
        torch.onnx.export(
            self.model,
            (dummy_points, dummy_density),
            output_path,
            input_names=['points', 'density'],
            output_names=['detections'],
            dynamic_axes={
                'points': {0: 'batch_size'},
                'density': {0: 'batch_size'},
                'detections': {0: 'batch_size'}
            }
        )

    def export_to_tensorrt(self, onnx_path, output_path):
        """
        导出模型到 TensorRT 格式（加速推理）
        """
        # TensorRT 导出逻辑
        # ...
        pass
```

## 功能流程梳理

### 流程 1: 深度图像转点云

```
深度相机 (RealSense/Kinect)
        ↓
  订阅深度图像话题
        ↓
  获取相机内参
        ↓
DepthToPointCloud::convert()
        ↓
  [深度图像 + 相机内参] → [X = (u-cx)*z/fx, Y = (v-cy)*z/fy, Z = z]
        ↓
  输出: pcl::PointCloud<PointXYZ>
```

### 流程 2: 传统点云物体检测

```
输入点云 (N points)
        ↓
  [可选] 坐标变换 (camera → base)
        ↓
  PassThroughFilter (Z轴截取)
        ↓
  VoxelFilter (下采样)
        ↓
  StatisticalFilter (去噪)
        ↓
  PlaneSegmentation (检测平面/地面)
        ↓
  从点云中移除平面内点
        ↓
  EuclideanClustering (聚类剩余点)
        ↓
  对每个聚类计算包围盒和中心点
        ↓
  输出: 物体列表 {center, size, point_count}
```

### 流程 3: 密度计算流程

```
输入点云 (N points)
        ↓
DensityCalculator::computeDensityFast()
        ↓
  [并行] 构建 KD 树
        ↓
  [并行] 对每个点查询 k 近邻
        ↓
  [并行] 计算高斯核密度
        ↓
  f(x) = (1/(n*h)) * Σ K((x-xi)/h)
        ↓
DensityCalculator::normalizeDensity() (MINMAX)
        ↓
  density = (density - min) / (max - min)  → [0, 1]
        ↓
  保存: density.npy 或 density.bin
        ↓
  输出: std::vector<float> (N values)
```

### 流程 4: 基于 AI 的 3D 目标检测 (离线流程)

```
数据准备阶段
====================
  [数据集] ScanNetV2 / SUN RGB-D
        ↓
  从网格采样点云 (10000 points/scene)
        ↓
  计算每个点的密度 (KDE)
        ↓
  提取 3D 边界框标注
        ↓
  保存: {points.npy, density.npy, labels.json}

训练阶段
==========
  加载训练数据集
        ↓
  for epoch in 180:
        for batch in dataloader:
            points, density, labels = batch
            ↓
            # 前向传播
            [points + density] → DensitySetAbstraction (4层)
            ↓
            [下采样后特征] → FeaturePropagation (2层)
            ↓
            [恢复特征] → CGNL 模块
            ↓
            [融合后特征] → VoteNetModule
            ↓
            # 后处理
            投票点 → 采样 → 聚类 → Proposal 回归
            ↓
            # 计算损失
            loss = {vote, objectness, class, box}
            ↓
            # 反向传播
            optimizer.step()
        ↓
    验证并计算 mAP
    ↓
  保存模型: model_epoch_xxx.pth

评估阶段
==========
  加载测试数据集
        ↓
  for scene in test_set:
        predictions = model(points, density)
        ↓
    DetectionEvaluator::evaluate()
        ↓
    计算 mAP@0.25, mAP@0.5
```

### 流程 5: 基于 AI 的 3D 目标检测 (在线推理)

```
ROS 订阅深度图像
        ↓
DepthToPointCloud (深度图 → 点云)
        ↓
  实时计算密度 (可选，或使用缓存)
        ↓
  下采样到 10000 points
        ↓
  [GPU] 加载预训练模型
        ↓
  [GPU] ObjectDetection3D::detect()
        ↓
    [Layer 1-4] DensitySetAbstraction (FPS + 密度融合)
        ↓
    [Layer 5-6] FeaturePropagation (上采样)
        ↓
    [Layer 7] CGNL (局部特征融合)
        ↓
    [Layer 8] VoteNet (投票)
        ↓
    [后处理] 聚类 → Proposal
        ↓
  输出: Detection[]
        ↓
  发布 3D 边界框到 ROS 话题
        ↓
  RViz 可视化
```

### 流程 6: 完整处理管道

```
输入: 深度图像 + 相机内参
        ↓
  ├─ 深度图转点云
  │
  ├─ [可选] 坐标变换
  │
  ├─ 空间滤波 (Z轴范围截取)
  │
  ├─ 体素滤波 (降采样)
  │
  ├─ 统计滤波 (去噪)
  │
  ├─ [分支 A: 传统方法]
  │      ├─ 平面分割
  │      └─ 欧式聚类
  │           ↓
  │      输出: 物体列表 (传统)
  │
  └─ [分支 B: AI 方法]
         ├─ 计算密度
         ├─ 下采样到 10000 点
         └─ 3D 检测网络
              ↓
         输出: 检测框 (AI)
              ↓
  融合结果 (可选)
        ↓
  可视化/发布
```

## 数据流图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          输入层                                │
├─────────────────────────────────────────────────────────────────────────┤
│  深度图像          │  相机内参          │  RGB 图像 (可选)    │
│  /depth/image_raw  │  /camera_info       │  /color/image_raw    │
└──────────┬────────────────┴──────────┬────────────────┴──────────┬──────────┘
           │                       │                       │
           ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      预处理层                                    │
├─────────────────────────────────────────────────────────────────────────┤
│  混合通道 (RGBD)          │  深度缩放 (×0.001)          │
│  ─────────────────────────   │  ─────────────────────────         │
│  │ 深度图转点云 │           │  │ 无效值过滤 │               │
│  └────────────────────────   │  └────────────────────────         │
└──────────┬──────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      滤波层                                      │
├─────────────────────────────────────────────────────────────────────────┤
│  坐标变换 (camera→base)  │  空间截取 (ROI)  │  体素滤波      │
│  ──────────────────────────  │  ───────────────  │  ────────────  │
│  统计滤波 (去噪)          │  半径滤波        │  密度计算     │
│  ──────────────────────────  │  ───────────────  │  ────────────  │
└──────────┬──────────────────────────────────────────────────────────┘
           │
           ├─────────────────────────────────────┐
           │                                      │
           ▼                                      ▼
┌─────────────────────────────────┐    ┌─────────────────────────────────┐
│      传统检测分支              │    │       AI 检测分支            │
├─────────────────────────────────┤    ├─────────────────────────────────┤
│  1. 平面分割 (RANSAC)         │    │  1. 密度归一化              │
│  2. 移除平面内点             │    │  2. 点云下采样                │
│  3. 欧式聚类                 │    │  3. 加载深度学习模型          │
│  4. 计算聚类中心             │    │  4. DensitySetAbstraction(×4) │
│  5. 生成包围盒              │    │  5. FeaturePropagation(×2)   │
└──────────┬────────────────────┘    │  6. CGNL 模块                │
           │                          │  7. VoteNet 投票             │
           │                          │  8. Proposal 回归            │
           │                          └──────────┬───────────────────┘
           │                                     │
           └───────────────┬───────────────────────┘
                         │
                         ▼
┌─────────────────────────────────┐
│         输出层                │
├─────────────────────────────────┤
│  ROS 话题发布:               │
│  - /detected_objects          │
│  - /processed_pointcloud       │
│  - /detection_markers         │
└─────────────────────────────────┘
```

## 关键算法说明

### 1. 最远点采样 (FPS)

**目的**: 均匀采样点云，保留全局结构信息

**算法**:
```
1. 随机选择第一个采样点
2. 计算所有点到已采样点的最小距离
3. 选择距离最远的点作为下一个采样点
4. 重复 2-3 直到采样 N 个点
```

**时间复杂度**: O(N×npoint)

### 2. 核密度估计 (KDE)

**目的**: 估计每个点周围的点云密度

**高斯核公式**:
```
f(x) = (1/(n×h)) × Σ_i K((x - xi)/h)

其中:
- K(u) = (1/√(2π)) × exp(-u²/2)  (标准高斯核)
- h: 带宽参数
- n: 点云总点数
```

### 3. 紧凑型广义非局部模块 (CGNL)

**目的**: 建立点之间的长距离依赖关系

**计算流程**:
```
1. 输入特征 X: (B, C, M)
2. 计算 θ = Conv(X), φ = Conv(X), g = Conv(X)
3. 分组减少计算量: θ, φ, g → (B, g, C/g, M)
4. 计算相似性: sim = softmax(θ × φᵀ / √(C/g))
5. 加权聚合: out = sim × g
6. 残差连接: Y = Conv(out) + X
```

### 4. VoteNet 投票机制

**目的**: 通过点投票找到物体中心

**流程**:
```
1. 每个点预测:
   - 偏移到物体中心 (dx, dy, dz)
   - 目标性分数 (是否属于某物体)
   - 类别分数

2. 选择目标性分数最高的 K 个投票点

3. 对投票点进行空间聚类

4. 对每个聚类:
   - 聚合特征
   - 回归最终 3D 框参数
```

## 配置文件格式 (model_config.yaml)

```yaml
# 模型配置
model:
  name: "DensityFusion3DNet"
  backbone: "pointnet2"
  num_classes: 18
  input_points: 10000
  feature_dim: 256
  num_proposals: 256

  # 网络结构
  layers:
    - type: "density_sa"
      npoint: 2048
      radius: 0.2
      nsample: 64
      mlp: [64, 64, 128]
      use_density: true
      density_mlp: [64, 128]
    - type: "density_sa"
      npoint: 512
      radius: 0.4
      nsample: 64
      mlp: [128, 128, 256]
      use_density: true
      density_mlp: [128, 256]
    - type: "density_sa"
      npoint: 128
      radius: 0.8
      nsample: 64
      mlp: [256, 256, 512]
      use_density: true
      density_mlp: [256, 512]
    - type: "density_sa"
      npoint: 1
      radius: 1.2
      nsample: 64
      mlp: [512, 512, 1024]
      use_density: true
      density_mlp: [512, 1024]

    - type: "feature_propagation"
      mlp: [256, 256]
    - type: "feature_propagation"
      mlp: [256, 256]

  # CGNL 模块
  cgnl:
    enabled: true
    in_channels: 256
    groups: 4

  # VoteNet 模块
  vote_net:
    num_proposals: 256
    vote_aggregation_radius: 0.3
    use_seed_features: true

# 训练配置
training:
  batch_size: 4
  num_epochs: 180
  learning_rate: 0.001
  weight_decay: 0.0001
  lr_decay:
    type: "step"
    step_size: 60
    gamma: 0.1

  # 损失权重
  loss_weights:
    vote: 1.0
    objectness: 5.0
    classification: 1.0
    box: 1.0

  # 数据增强
  augmentation:
    random_rotation: true
    rotation_range: [-180, 180]
    random_scaling: true
    scale_range: [0.8, 1.2]
    random_flip: true
    jitter: true
    jitter_std: 0.01
    point_dropout: true
    dropout_prob: 0.1

# 数据集配置
dataset:
  name: "scannet"
  root: "/path/to/scannet"
  num_points: 10000
  density_bandwidth: 0.5
  density_precompute: true
  use_color: false

# 评估配置
evaluation:
  iou_thresholds: [0.25, 0.5]
  confidence_threshold: 0.05
  max_detections_per_image: 100
```

## 包结构设计

```
src/pcl_processing/
├── include/pcl_processing/           # C++ 头文件
│   ├── depth_to_pointcloud.h        # 深度图转点云
│   ├── density_calculator.h         # 密度计算
│   ├── filters/                     # 滤波器模块
│   │   ├── voxel_filter.h
│   │   ├── statistical_filter.h
│   │   ├── passthrough_filter.h
│   │   └── radius_filter.h
│   ├── segmentation/                # 分割模块
│   │   ├── plane_segmentation.h
│   │   └── euclidean_clustering.h
│   ├── pointcloud_transformer.h     # 坐标变换
│   ├── pointcloud_visualizer.h      # 可视化
│   ├── pointcloud_lut.h             # 查找表
│   ├── pointcloud_pipeline.h        # 处理管道
│   ├── camera_intrinsics.h          # 相机内参
│   └── common.h                     # 公共定义
├── src/                             # C++ 源文件
│   ├── depth_to_pointcloud.cpp
│   ├── density_calculator.cpp
│   ├── filters/
│   ├── segmentation/
│   └── ...
├── python/pcl_processing/           # Python 接口
│   ├── __init__.py
│   ├── depth_to_pointcloud.py
│   ├── density_calculator.py
│   ├── filters.py
│   └── detection/                  # 3D 检测模块
│       ├── __init__.py
│       ├── model_config.py
│       ├── object_detection_3d.py  # 主检测器
│       ├── modules/                 # 网络子模块
│       │   ├── __init__.py
│       │   ├── density_sa.py       # 密度融合下采样
│       │   ├── cgnl.py            # CGNL 模块
│       │   ├── vote_net.py         # 投票网络
│       │   └── feature_propagation.py
│       ├── training/                # 训练模块
│       │   ├── __init__.py
│       │   ├── trainer.py
│       │   ├── loss.py
│       │   └── evaluator.py
│       ├── data/                   # 数据模块
│       │   ├── __init__.py
│       │   ├── dataset.py
│       │   └── augmentation.py
│       ├── utils/                  # 工具函数
│       │   ├── __init__.py
│       │   ├── pointnet_utils.py   # FPS、索引等工具
│       │   ├── visualization.py
│       │   └── export.py
│       └── configs/                # 配置文件
│           ├── sunrgbd.yaml
│           └── scannet.yaml
├── examples/                        # 示例程序
│   ├── example_depth_to_pc.cpp      # 深度图转点云示例
│   ├── example_filters.cpp          # 滤波器示例
│   ├── example_segmentation.cpp     # 分割示例
│   ├── example_pipeline.cpp         # 管道示例
│   ├── example_density_calc.cpp     # 密度计算示例
│   └── example_3d_detection.py    # 3D 检测示例
├── launch/                          # Launch 文件
│   ├── pcl_processing_demo.launch.py
│   └── detection_demo.launch.py
├── config/                          # 配置文件
│   ├── pipeline_config.yaml
│   └── model_config.yaml
├── models/                          # 预训练模型
│   ├── README.md
│   └── checkpoints/
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 依赖关系

### C++ 依赖
- ROS2 Humble 核心: `rclcpp`, `sensor_msgs`, `geometry_msgs`
- PCL: `pcl_common`, `pcl_filters`, `pcl_segmentation`, `pcl_ros`
- OpenCV: `cv_bridge`, `OpenCV`
- TF: `tf2_ros`, `tf2_geometry_msgs`
- Eigen: `Eigen3`

### Python 依赖
- `numpy`
- `opencv-python`
- `open3d` (可选，用于高级可视化)
- `torch` >= 2.0 (深度学习框架)
- `torchvision` (可选，数据增强)
- `scikit-learn` (KDE 计算)
- `tensorboard` (可选，训练监控)
- `pyyaml` (配置文件解析)
- `onnx` (可选，模型导出)
- `tensorrt` (可选，推理加速)

### 硬件要求
- **基础功能**: CPU 即可
- **深度学习推理**: GPU 显存 >= 8GB (推荐 NVIDIA RTX 2070 Super 或以上)
- **深度学习训练**: GPU 显存 >= 12GB (推荐 NVIDIA RTX 3090 或以上)

## 示例用法

### 示例 1: 深度图转点云
```cpp
#include <pcl_processing/depth_to_pointcloud.h>

// 相机内参
CameraIntrinsics intrinsics;
intrinsics.fx = 615.0f;
intrinsics.fy = 615.0f;
intrinsics.cx = 320.0f;
intrinsics.cy = 240.0f;

DepthToPointCloud converter(intrinsics);
auto pointcloud = converter.convert(depth_image);
```

### 示例 2: 使用滤波器
```cpp
#include <pcl_processing/filters/voxel_filter.h>
#include <pcl_processing/filters/statistical_filter.h>

VoxelFilter voxel_filter(0.01f);
StatisticalFilter stat_filter(50, 1.0f);

auto filtered = stat_filter.filter(voxel_filter.filter(raw_cloud));
```

### 示例 3: 物体检测
```cpp
#include <pcl_processing/segmentation/plane_segmentation.h>
#include <pcl_processing/segmentation/euclidean_clustering.h>

PlaneSegmentation plane_seg(0.05f);
EuclideanClustering clustering(0.1f, 100, 25000);

// 移除平面
auto plane_result = plane_seg.segment(cloud);

// 聚类剩余点（物体）
auto objects = clustering.cluster(plane_result.inliers);
```

### 示例 4: 完整处理管道
```cpp
#include <pcl_processing/pointcloud_pipeline.h>

PointCloudPipeline pipeline;
pipeline.addFilter(std::make_shared<VoxelFilter>(0.01f));
pipeline.addFilter(std::make_shared<StatisticalFilter>(50, 1.0f));
pipeline.addFilter(std::make_shared<PassThroughFilter>());
pipeline.addSegmentation(std::make_shared<PlaneSegmentation>(0.05f));

auto result = pipeline.process(raw_cloud);
```

## 配置文件格式 (pipeline_config.yaml)

```yaml
pipeline:
  name: "object_detection_pipeline"
  steps:
    - type: "voxel_filter"
      params:
        leaf_size: 0.01
    - type: "statistical_filter"
      params:
        mean_k: 50
        std_dev: 1.0
    - type: "passthrough_filter"
      params:
        field_name: "z"
        min: 0.5
        max: 2.0
    - type: "plane_segmentation"
      params:
        distance_threshold: 0.05
    - type: "euclidean_clustering"
      params:
        cluster_tolerance: 0.1
        min_cluster_size: 100
        max_cluster_size: 25000
```

## 性能优化建议

1. **点云下采样**: 体素滤波在处理开始时进行，减少后续计算量
2. **并行处理**: 使用 OpenMP 并行处理点云滤波和分割
3. **GPU 加速**: 考虑使用 CUDA 加速密集计算（可选）
4. **零拷贝**: 使用智能指针避免不必要的数据拷贝
5. **KD 树缓存**: 聚类分割时重用 KD 树

## 与现有代码的兼容性

本设计不会修改现有功能包（`wpr_simulation2`, `hand_eyes_calibration`, `ros2_dummy_arm_810`），但可以逐步迁移它们的点云处理逻辑到新包中。

| 现有代码 | 新包映射 |
|---------|---------|
| `wpr_simulation2/10_pc_objects.cpp` | `Pipeline` + `PlaneSegmentation` + `EuclideanClustering` |
| `hand_eyes_calibration/pc_LUT.h` | `PointCloudLUT` |
| `ros2_dummy_arm_810/depth_image_visualizer.py` | `DepthToPointCloud` + `Visualizer` |

## 测试计划

### 单元测试
- 每个滤波器的正确性测试
- 深度图转点云的精度测试
- 坐标变换的正确性测试

### 集成测试
- 完整处理管道测试
- 与真实相机数据测试
- 仿真环境测试

### 性能测试
- 不同规模点云的处理时间
- 内存占用测试
- 端到端延迟测试

## 开发计划阶段

### 阶段 1: 核心功能实现 (C++)
1. ✅ 包结构搭建
2. 深度图转点云模块
3. 基础滤波器（体素、统计、空间、半径）
4. 坐标变换模块
5. 可视化模块
6. 测试和文档

### 阶段 2: 高级功能实现 (C++)
1. 分割模块（平面、聚类）
2. 查找表模块
3. 密度计算模块
4. 处理管道模块
5. Python 接口绑定
6. 测试和文档

### 阶段 3: 深度学习模块 (Python)
1. 网络子模块实现
   - DensitySetAbstraction (密度融合下采样)
   - FeaturePropagation (特征传播)
   - CGNL (紧凑型广义非局部网络)
   - VoteNetModule (投票网络)
2. 训练与评估模块
3. 数据加载与增强
4. 可视化工具

### 阶段 4: 集成与优化
1. 端到端系统集成
2. ROS 节点封装
3. 性能优化（CUDA 加速、并行处理）
4. 模型导出（ONNX/TensorRT）
5. 完整测试和文档完善

## 编译与安装

```bash
# 编译
colcon build --symlink-install --packages-select pcl_processing

# 加载环境
source install/setup.bash

# 运行示例
ros2 run pcl_processing example_depth_to_pc
```

## 注意事项

1. RealSense D435 深度图需要乘以 0.001 转换为米
2. 不同相机内参需要正确配置
3. TF 变换需要确保时钟同步
4. 大规模点云处理建议使用体素滤波先降采样
