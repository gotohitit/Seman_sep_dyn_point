# 将PCD文件中的动态点标注出来

### 1. **读取点云文件**

- 程序从`../PCD`文件夹中读取一个指定名称的PCD文件（如`input.pcd`）。PCD（Point Cloud Data）是一种用于存储三维点云数据的文件格式，通常由激光雷达或其他传感器生成。
- 通过使用PCL（Point Cloud Library）的`loadPCDFile`函数，程序将PCD文件加载到`pcl::PointCloud<pcl::PointXYZI>`类型的点云对象中。这里`pcl::PointXYZI`表示每个点包含x、y、z坐标和intensity（强度）信息。

### 2. **遍历点云并修改强度值**

- 程序遍历点云中的每个点，读取每个点的`intensity`（强度）值。
- 通过位运算，将每个点的强度值的低16位提取为label，并根据label的值范围修改点的强度：
  - 如果`label`在252到259之间，将该点的强度值设为`240`。
  - 否则，将该点的强度值设为`10`。
- 此操作的目的是对点云数据进行某种分类或过滤，将符合特定`label`条件的点标记为一种强度值，其他点则标记为另一种强度值。

### 3. **保存修改后的点云文件**

- 程序将修改后的点云数据保存为新的PCD文件，文件名为`modified_<filename>`，并存储在同一个`../PCD_OUT`目录中。
- 例如，如果输入文件名为`input.pcd`，输出的修改文件名为`modified_input.pcd`。



### 编译和运行步骤

1. 创建构建目录：

   ```
   mkdir build
   cd build
   ```

2. 运行CMake生成Makefile：

   ```
   cmake ..
   ```

3. 编译代码：

   ```
   make
   ```

4. 运行程序（假设PCD文件名为`input.pcd`）：

   在目录下创建两个文件夹，一个PCD，一个PCD_OUT，将要处理的pcd文件放到PCD文件夹下。

   ```
   ./ModifyIntensity input.pcd
   ```