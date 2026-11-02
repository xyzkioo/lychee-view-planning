# 第三方模型与外部依赖

荔枝树原素材来自 [CadNav Lychee tree with fruit](https://www.cadnav.com/3d-models/model-31586.html)。原网页标注 Non-commercial，压缩包内说明措辞不同。本项目按较严格的非商业研究用途处理，未将原素材作为可自由再分发的数据集。

本仓库只包含本项目的小车模型、控制/实验代码及已有实验渲染示例，不包含第三方树网格、贴图、MAX/Blender 源文件或转换器源码。使用者需独立获得合适使用权，并提供 Gazebo 可加载的 `lychee_tree` 模型。

原开发环境通过 [nrgsille76/io_scene_max](https://github.com/nrgsille76/io_scene_max) 将 MAX 导入 Blender，再导出 OBJ、整理材质并封装为 SDF；该导入器原有许可另行适用。树被设为名义 3 米高，部分果实材质使用统一红色替代。显示网格包括枝叶，但物理碰撞仅为简化树干。

运行依赖包括 Gazebo Harmonic、Gazebo Transport/Messages Python 绑定和 Pillow，均应遵守各自许可。本仓库的上传不改变这些第三方组件的许可。
