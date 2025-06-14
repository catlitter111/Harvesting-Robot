// pages/detection/detection.js
// 智能水果识别页面 - 核心逻辑实现

/**
 * 全局消息ID计数器，确保每个检测记录的唯一性
 */
let detectionIdCounter = 0;

/**
 * 生成唯一的检测ID
 * @param {string} prefix - ID前缀
 * @returns {string} 唯一ID
 */
function generateDetectionId(prefix = 'detection') {
  detectionIdCounter++;
  const timestamp = Date.now();
  const random = Math.random().toString(36).substr(2, 9);
  const counter = detectionIdCounter.toString().padStart(5, '0');
  return `${prefix}_${timestamp}_${counter}_${random}`;
}

Page({
  /**
   * 页面的初始数据
   */
  data: {
    // === AI服务状态 ===
    aiServiceStatus: 'offline', // 'online' | 'offline' | 'connecting'
    
    // === 检测统计数据 ===
    todayDetectionCount: 0,     // 今日检测次数
    
    // === 检测模式相关 ===
    detectionMode: 'comprehensive',  // 'comprehensive' | 'maturity' | 'health' | 'quality'
    detectionModeText: '综合检测',   // 显示的模式文本
    detectionModeMap: {
      'comprehensive': '综合检测',
      'maturity': '成熟度检测', 
      'health': '健康检测',
      'quality': '品质评估'
    },
    
    // === 当前图片相关 ===
    currentImage: '',           // 当前选择的图片路径
    imageName: '',              // 图片名称
    imageTime: '',              // 图片选择时间
    
    // === 检测状态 ===
    detecting: false,           // 是否正在检测中
    
    // === 检测结果数据 ===
    detectionResult: null,      // 当前检测结果
    /*
    检测结果数据结构示例：
    {
      id: 'detection_xxx',
      fruitType: '红富士苹果',
      fruitEmoji: '🍎',
      variety: '红富士',
      confidence: 95,
      maturity: 85,
      healthStatus: '健康',
      healthGrade: 'good',
      pestStatus: 'none',
      diseaseStatus: 'none',
      qualityScore: 92,
      appearanceStars: 4,
      sizeCategory: '中等',
      overallGrade: 'Excellent',
      recommendation: '该苹果成熟度适中，无病虫害，建议立即采摘。',
      suggestedAction: 'harvest', // 'harvest' | 'wait' | 'inspect'
      actionable: true,
      boundingBox: { x: 100, y: 50, width: 200, height: 150 },
      timestamp: 1640995200000
    }
    */
    
    // === 历史记录相关 ===
    detectionHistory: [],       // 检测历史记录
    filterDate: '',             // 筛选日期
    filterDateText: '今日',     // 筛选日期显示文本
    hasMoreHistory: false,      // 是否有更多历史记录
    loadingMoreHistory: false,  // 是否正在加载更多
    
    // === 历史统计数据 ===
    historyStats: {
      totalDetections: 0,       // 总检测次数
      excellentCount: 0,        // 优质果数量
      averageMaturity: 0,       // 平均成熟度
      accuracyRate: 0           // 准确率
    },
    
    // === UI控制相关 ===
    showImageOptions: false,    // 是否显示图片选择选项
    showModeModal: false,       // 是否显示模式选择模态框
    
    // === 设备和权限 ===
    cameraAuthorized: false,    // 相机权限
    albumAuthorized: false,     // 相册权限
    
    // === WebSocket连接状态 ===
    connected: false,           // WebSocket连接状态
    robotId: 'robot_123',       // 机器人ID
    
    // === 本地图片存储配置 ===
    localImageStorage: true,    // 启用本地图片存储
    imageStorageDir: 'fruit_images',  // 本地图片存储目录
    maxLocalImages: 50,         // 最大本地图片数量
    
    // === 预设水果图片库 ===
    fruitImageLibrary: {
      '嘎啦苹果': '/images/fruits/apple_gala.jpg',
      '红富士苹果': '/images/fruits/apple_fuji.jpg', 
      '青苹果': '/images/fruits/apple_green.jpg',
      '橙子': '/images/fruits/orange.jpg',
      '柠檬': '/images/fruits/lemon.jpg',
      '香蕉': '/images/fruits/banana.jpg',
      '草莓': '/images/fruits/strawberry.jpg',
      '葡萄': '/images/fruits/grape.jpg',
      '桃子': '/images/fruits/peach.jpg',
      '梨': '/images/fruits/pear.jpg',
      '樱桃': '/images/fruits/cherry.jpg',
      '猕猴桃': '/images/fruits/kiwi.jpg',
      '芒果': '/images/fruits/mango.jpg',
      '菠萝': '/images/fruits/pineapple.jpg',
      '西瓜': '/images/fruits/watermelon.jpg',
      '甜橙': '/images/fruits/sweet_orange.jpg'
    },
    
    // === 服务器配置 ===
    serverBaseUrl: 'http://10.52.163.144:8000',  // 服务器基础URL - 请根据实际情况修改IP地址
  },

  /**
   * 生命周期函数--监听页面加载
   */
  onLoad: function (options) {
    console.log('Detection页面加载');
    
    // 初始化页面
    this.initializePage();
    
    // 检查设备权限
    this.checkDevicePermissions();
    
    // 注册到全局应用以接收WebSocket消息
    this.registerToGlobalApp();
    
    // 加载本地数据
    this.loadLocalData();
    
    // 检查AI服务状态
    this.checkAIServiceStatus();
    
    // 清理过期的本地图片
    this.cleanupLocalImages();
  },

  /**
   * 生命周期函数--监听页面显示
   */
  onShow: function () {
    console.log('Detection页面显示');
    
    // 重新注册到全局应用
    this.registerToGlobalApp();
    
    // 更新连接状态
    this.updateConnectionStatus();
    
    // 刷新今日统计
    this.refreshTodayStats();
  },

  /**
   * 生命周期函数--监听页面隐藏
   */
  onHide: function () {
    console.log('Detection页面隐藏');
    
    // 清理定时器
    this.clearTimers();
  },

  /**
   * 生命周期函数--监听页面卸载
   */
  onUnload: function () {
    console.log('Detection页面卸载');
    
    // 取消全局注册
    const app = getApp();
    if (app.globalData) {
      app.globalData.detectionPage = null;
    }
    
    // 清理资源
    this.clearTimers();
    this.clearTempImages();
  },

  /**
   * 页面相关事件处理函数--监听用户下拉动作
   */
  onPullDownRefresh: function () {
    console.log('用户下拉刷新');
    
    // 刷新页面数据
    this.refreshPageData().finally(() => {
      wx.stopPullDownRefresh();
    });
  },

  // ==================== 初始化相关方法 ====================

  /**
   * 初始化页面数据和状态
   */
  initializePage: function() {
    // 设置初始筛选日期为今天
    const today = new Date();
    const todayStr = this.formatDate(today);
    
    this.setData({
      filterDate: todayStr,
      filterDateText: '今日',
      // 初始化空的统计数据，等待从服务器加载
      historyStats: {
        totalDetections: 0,
        excellentCount: 0,
        averageMaturity: 0,
        accuracyRate: 0
      }
    });
    
    console.log('页面初始化完成');
  },

  /**
   * 注册页面到全局应用，用于接收WebSocket消息
   */
  registerToGlobalApp: function() {
    const app = getApp();
    if (app.globalData) {
      app.globalData.detectionPage = this;
      
      // 更新连接状态
      this.setData({
        connected: app.globalData.connected || false
      });
      
      // 如果WebSocket已连接，立即请求历史记录
      if (app.globalData.connected) {
        this.requestDetectionHistoryFromServer();
      }
    }
  },

  /**
   * 检查设备权限（相机和相册）
   */
  checkDevicePermissions: function() {
    const that = this;
    
    // 检查相机权限和相册权限
    wx.getSetting({
      success: (res) => {
        that.setData({
          cameraAuthorized: !!res.authSetting['scope.camera'],
          albumAuthorized: !!res.authSetting['scope.album'] // 修改为 scope.album
        });
      },
      fail: (error) => {
        console.error('获取设备权限状态失败:', error);
      }
    });
  },

  /**
   * 加载本地存储的数据
   */
  loadLocalData: function() {
    try {
      // 加载检测历史
      let history = wx.getStorageSync('detection_history') || [];
      
      // 如果没有本地历史数据，从服务器获取
      if (history.length === 0) {
        this.requestDetectionHistoryFromServer();
      }
      
      // 加载今日统计
      const todayStats = wx.getStorageSync('today_detection_stats') || {
        date: this.formatDate(new Date()),
        count: 0
      };
      
      // 如果不是今天的数据，重置统计
      const today = this.formatDate(new Date());
      if (todayStats.date !== today) {
        todayStats.date = today;
        todayStats.count = 0;
        wx.setStorageSync('today_detection_stats', todayStats);
      }
      
      // 加载检测模式设置
      const savedMode = wx.getStorageSync('detection_mode') || 'comprehensive';
      
      this.setData({
        detectionHistory: history.slice(0, 20), // 只显示最近20条
        todayDetectionCount: todayStats.count,
        detectionMode: savedMode,
        detectionModeText: this.data.detectionModeMap[savedMode] || '综合检测',
        hasMoreHistory: history.length > 20
      });
      
      // 计算历史统计
      this.calculateHistoryStats(history);
      
      console.log('本地数据加载完成', {
        historyCount: history.length,
        todayCount: todayStats.count,
        mode: savedMode
      });
      
    } catch (error) {
      console.error('加载本地数据失败:', error);
    }
  },

  // ==================== AI服务相关方法 ====================

  /**
   * 检查AI服务状态
   */
  checkAIServiceStatus: function() {
    const app = getApp();
    
    if (!app.globalData || !app.globalData.connected) {
      this.setData({ aiServiceStatus: 'offline' });
      return;
    }
    
    // 发送AI服务状态检查请求
    this.setData({ aiServiceStatus: 'connecting' });
    
    try {
      app.sendSocketMessage({
        type: 'check_ai_service',
        timestamp: Date.now()
      });
      
      // 5秒后如果没有响应，认为服务离线
      setTimeout(() => {
        if (this.data.aiServiceStatus === 'connecting') {
          this.setData({ aiServiceStatus: 'offline' });
        }
      }, 5000);
      
    } catch (error) {
      console.error('检查AI服务状态失败:', error);
      // 开发阶段设置为在线，方便测试界面
      this.setData({ aiServiceStatus: 'online' });
    }
  },

  /**
   * 处理AI服务状态响应
   * 服务端接口：当收到check_ai_service请求时，返回ai_service_status响应
   */
  handleAIServiceStatus: function(data) {
    const status = data.available ? 'online' : 'offline';
    this.setData({ aiServiceStatus: status });
    
    console.log('AI服务状态更新:', status);
  },

  // ==================== 图片选择相关方法 ====================

  /**
   * 显示图片选择选项
   */
  showImageOptions: function() {
    this.setData({ showImageOptions: true });
  },

  /**
   * 隐藏图片选择选项
   */
  hideImageOptions: function() {
    this.setData({ showImageOptions: false });
  },

  /**
   * 阻止事件冒泡
   */
  stopPropagation: function() {
    // 空函数，用于阻止事件冒泡
  },

  /**
   * 拍照选择图片
   */
  takePhoto: function() {
    const that = this;
    
    this.hideImageOptions();
    
    // 检查相机权限
    if (!this.data.cameraAuthorized) {
      this.requestCameraPermission(() => {
        that.takePhotoImpl();
      });
    } else {
      this.takePhotoImpl();
    }
  },

  /**
   * 实际执行拍照操作
   */
  takePhotoImpl: function() {
    const that = this;
    
    wx.chooseImage({
      count: 1,
      sizeType: ['compressed'], // 使用压缩图
      sourceType: ['camera'],   // 只能拍照
      success: (res) => {
        const imagePath = res.tempFilePaths[0];
        that.handleImageSelected(imagePath, 'camera');
      },
      fail: (error) => {
        console.error('拍照失败:', error);
        that.showErrorToast('拍照失败，请重试');
      }
    });
  },

  /**
   * 从相册选择图片
   */
  chooseFromAlbum: function() {
    const that = this;
    
    this.hideImageOptions();
    
    // 检查相册权限
    if (!this.data.albumAuthorized) {
      this.requestAlbumPermission(() => {
        that.chooseFromAlbumImpl();
      });
    } else {
      this.chooseFromAlbumImpl();
    }
  },

  /**
   * 实际执行从相册选择操作
   */
  chooseFromAlbumImpl: function() {
    const that = this;
    
    wx.chooseImage({
      count: 1,
      sizeType: ['compressed'], // 使用压缩图
      sourceType: ['album'],    // 只能从相册选择
      success: (res) => {
        const imagePath = res.tempFilePaths[0];
        that.handleImageSelected(imagePath, 'album');
      },
      fail: (error) => {
        console.error('选择图片失败:', error);
        that.showErrorToast('选择图片失败，请重试');
      }
    });
  },

  /**
   * 批量选择图片（未来功能）
   */
  chooseMultipleImages: function() {
    this.hideImageOptions();
    
    wx.showToast({
      title: '批量检测功能即将上线',
      icon: 'none',
      duration: 2000
    });
  },

  /**
   * 处理选中的图片
   * @param {string} imagePath - 图片路径
   * @param {string} source - 图片来源 'camera' | 'album'
   */
  handleImageSelected: function(imagePath, source = 'unknown') {
    const that = this;
    
    console.log('图片选择成功:', imagePath, source);
    
    // 显示加载提示
    wx.showLoading({
      title: '处理图片中...',
      mask: true
    });
    
    // 压缩和处理图片
    this.processImage(imagePath)
      .then((processedPath) => {
        // 生成图片信息
        const now = new Date();
        const imageName = `${source === 'camera' ? '拍摄' : '相册'}_${that.formatTime(now)}`;
        
        that.setData({
          currentImage: processedPath,
          imageName: imageName,
          imageTime: that.formatTime(now),
          detectionResult: null // 清除之前的检测结果
        });
        
        wx.hideLoading();
        
        // 自动开始检测（可选）
        setTimeout(() => {
          that.promptAutoDetection();
        }, 500);
        
      })
      .catch((error) => {
        wx.hideLoading();
        console.error('图片处理失败:', error);
        that.showErrorToast('图片处理失败，请重新选择');
      });
  },

  /**
   * 图片处理和压缩
   * @param {string} imagePath - 原始图片路径
   * @returns {Promise<string>} 处理后的图片路径
   */
  processImage: function(imagePath) {
    return new Promise((resolve, reject) => {
      const that = this;
      
      // 获取图片信息
      wx.getImageInfo({
        src: imagePath,
        success: (imageInfo) => {
          console.log('原图片信息:', imageInfo);
          
          // 计算压缩尺寸
          const maxWidth = 1024;
          const maxHeight = 1024;
          const quality = 0.8;
          
          let { width, height } = imageInfo;
          
          // 如果图片过大，按比例缩小
          if (width > maxWidth || height > maxHeight) {
            const ratio = Math.min(maxWidth / width, maxHeight / height);
            width = Math.floor(width * ratio);
            height = Math.floor(height * ratio);
          }
          
          // 使用Canvas进行压缩
          that.compressImageWithCanvas(imagePath, width, height, quality)
            .then(resolve)
            .catch(reject);
        },
        fail: (error) => {
          console.error('获取图片信息失败:', error);
          reject(error);
        }
      });
    });
  },

  /**
   * 使用Canvas 2D压缩图片
   * @param {string} imagePath - 图片路径
   * @param {number} width - 目标宽度
   * @param {number} height - 目标高度
   * @param {number} quality - 压缩质量 0-1
   * @returns {Promise<string>} 压缩后的图片路径
   */
  compressImageWithCanvas: function(imagePath, width, height, quality) {
    return new Promise((resolve, reject) => {
      // 使用新的Canvas 2D API
      const query = wx.createSelectorQuery().in(this);
      query.select('#imageProcessCanvas')
        .fields({ node: true, size: true })
        .exec((res) => {
          if (!res || !res[0]) {
            reject(new Error('Canvas节点获取失败'));
            return;
          }

          const canvas = res[0].node;
          const ctx = canvas.getContext('2d');

          // 设置canvas尺寸
          const dpr = wx.getSystemInfoSync().pixelRatio;
          canvas.width = width * dpr;
          canvas.height = height * dpr;
          ctx.scale(dpr, dpr);

          // 创建图片对象
          const img = canvas.createImage();
          img.onload = () => {
            // 清除画布
            ctx.clearRect(0, 0, width, height);
            // 绘制图片
            ctx.drawImage(img, 0, 0, width, height);

            // 导出压缩后的图片
            wx.canvasToTempFilePath({
              canvas: canvas,
              width: width,
              height: height,
              destWidth: width,
              destHeight: height,
              quality: quality,
              fileType: 'jpg',
              success: (res) => {
                console.log('图片压缩完成:', res.tempFilePath);
                resolve(res.tempFilePath);
              },
              fail: (error) => {
                console.error('图片压缩失败:', error);
                reject(error);
              }
            });
          };

          img.onerror = (error) => {
            console.error('图片加载失败:', error);
            reject(error);
          };

          // 设置图片源
          img.src = imagePath;
        });
    });
  },

  /**
   * 预览当前图片
   */
  previewImage: function() {
    if (!this.data.currentImage) return;
    
    wx.previewImage({
      urls: [this.data.currentImage],
      current: this.data.currentImage
    });
  },

  /**
   * 清除当前选择的图片
   */
  clearCurrentImage: function() {
    wx.showModal({
      title: '确认删除',
      content: '确定要删除当前选择的图片吗？',
      success: (res) => {
        if (res.confirm) {
          this.setData({
            currentImage: '',
            imageName: '',
            imageTime: '',
            detectionResult: null
          });
        }
      }
    });
  },

  // ==================== AI检测相关方法 ====================

  /**
   * 提示是否自动开始检测
   */
  promptAutoDetection: function() {
    if (this.data.aiServiceStatus !== 'online') {
      return;
    }
    
    wx.showModal({
      title: '开始检测',
      content: '图片已准备完成，是否立即开始AI智能检测？',
      confirmText: '立即检测',
      cancelText: '稍后检测',
      success: (res) => {
        if (res.confirm) {
          this.startDetection();
        }
      }
    });
  },

  /**
   * 开始AI检测
   */
  startDetection: function() {
    // 参数验证
    if (!this.data.currentImage) {
      this.showErrorToast('请先选择图片');
      return;
    }
    
    if (this.data.detecting) {
      console.log('正在检测中，忽略重复请求');
      return;
    }
    
    if (this.data.aiServiceStatus !== 'online') {
      this.showErrorToast('AI服务暂时不可用');
      return;
    }
    
    // 开始检测
    this.setData({ detecting: true });
    
    console.log('开始AI检测:', {
      image: this.data.currentImage,
      mode: this.data.detectionMode
    });
    
    // 将图片转换为base64并发送到服务端
    this.convertImageToBase64(this.data.currentImage)
      .then((base64Data) => {
        this.sendDetectionRequest(base64Data);
      })
      .catch((error) => {
        this.handleDetectionError(error);
      });
  },

  /**
   * 将图片转换为base64格式
   * @param {string} imagePath - 图片路径
   * @returns {Promise<string>} base64数据
   */
  convertImageToBase64: function(imagePath) {
    return new Promise((resolve, reject) => {
      wx.getFileSystemManager().readFile({
        filePath: imagePath,
        encoding: 'base64',
        success: (res) => {
          resolve(res.data);
        },
        fail: (error) => {
          console.error('转换图片为base64失败:', error);
          reject(error);
        }
      });
    });
  },

  /**
   * 发送检测请求到服务端
   * 服务端接口：ai_fruit_detection
   * @param {string} imageBase64 - 图片base64数据
   */
  sendDetectionRequest: function(imageBase64) {
    const app = getApp();
    
    if (!app.globalData || !app.globalData.connected) {
      this.handleDetectionError(new Error('WebSocket连接已断开'));
      return;
    }
    
    const requestData = {
      type: 'ai_fruit_detection',
      data: {
        image_base64: imageBase64,
        detection_mode: this.data.detectionMode,
        robot_id: this.data.robotId,
        timestamp: Date.now(),
        image_info: {
          name: this.data.imageName,
          time: this.data.imageTime
        }
      }
    };
    
    try {
      app.sendSocketMessage(requestData);
      console.log('检测请求已发送');
      
      // 设置超时处理
      this.detectionTimeout = setTimeout(() => {
        this.handleDetectionTimeout();
      }, 30000); // 30秒超时
      
    } catch (error) {
      console.error('发送检测请求失败:', error);
      this.handleDetectionError(error);
    }
  },

  /**
   * 处理检测超时
   */
  handleDetectionTimeout: function() {
    console.warn('AI检测请求超时');
    
    this.setData({ detecting: false });
    
    wx.showModal({
      title: '检测超时',
      content: 'AI检测服务响应超时，请检查网络连接或稍后重试',
      confirmText: '重试',
      cancelText: '取消',
      success: (res) => {
        if (res.confirm) {
          this.startDetection();
        }
      }
    });
  },

  /**
   * 处理检测成功响应
   * 服务端响应格式：ai_fruit_detection_response
   * @param {Object} data - 检测结果数据
   */
  handleDetectionResponse: function(data) {
    console.log('收到AI检测结果:', data);
    
    // 清除超时定时器
    if (this.detectionTimeout) {
      clearTimeout(this.detectionTimeout);
      this.detectionTimeout = null;
    }
    
    this.setData({ detecting: false });
    
    if (data.success) {
      // 处理成功结果
      const result = this.formatDetectionResult(data.result);
      
      this.setData({ detectionResult: result });
      
      // 保存检测记录
      this.saveDetectionRecord(result);
      
      // 更新统计数据
      this.updateTodayStats();
      
      // 显示成功提示
      wx.showToast({
        title: '检测完成',
        icon: 'success',
        duration: 1500
      });
      
    } else {
      // 处理失败结果
      this.handleDetectionError(new Error(data.message || '检测失败'));
    }
  },

  /**
   * 格式化检测结果数据
   * @param {Object} rawResult - 原始检测结果
   * @returns {Object} 格式化后的结果
   */
  formatDetectionResult: function(rawResult) {
    // 生成检测记录ID
    const detectionId = generateDetectionId();
    
    // 格式化结果数据（确保所有字符串字段都安全）
    const formattedResult = {
      id: detectionId,
      fruitType: this.safeString(rawResult.fruit_type, '未知水果'),
      fruitEmoji: this.getFruitEmoji(rawResult.fruit_type),
      variety: this.safeString(rawResult.variety, '未知品种'),
      confidence: Math.round(rawResult.confidence || 0),
      maturity: Math.round(rawResult.maturity_percentage || 0),
      healthStatus: this.safeString(rawResult.health_status, '未知'),
      healthGrade: this.safeString(this.getHealthGrade(rawResult.health_score), 'unknown'),
      pestStatus: this.safeString(rawResult.pest_status, 'none'),
      diseaseStatus: this.safeString(rawResult.disease_status, 'none'),
      qualityScore: Math.round(rawResult.quality_score || 0),
      appearanceStars: Math.round((rawResult.appearance_score || 0) / 20), // 转换为1-5星
      sizeCategory: this.safeString(rawResult.size_category, '中等'),
      overallGrade: this.safeString(this.getOverallGrade(rawResult.overall_score), 'Unknown'),
      recommendation: this.safeString(rawResult.recommendation, '暂无建议'),
      suggestedAction: this.safeString(rawResult.suggested_action, 'inspect'),
      actionable: rawResult.actionable !== false,
      boundingBox: rawResult.bounding_box || null,
      timestamp: Date.now(),
      imagePath: this.data.currentImage,
      imageName: this.data.imageName,
      detectionMode: this.data.detectionMode
    };
    
    console.log('格式化检测结果:', formattedResult);
    return formattedResult;
  },

  /**
   * 根据水果类型获取emoji表情
   * @param {string} fruitType - 水果类型
   * @returns {string} emoji表情
   */
  getFruitEmoji: function(fruitType) {
    const emojiMap = {
      '苹果': '🍎',
      '梨': '🍐', 
      '橙子': '🍊',
      '柠檬': '🍋',
      '香蕉': '🍌',
      '草莓': '🍓',
      '桃子': '🍑',
      '葡萄': '🍇'
    };
    
    for (const [key, emoji] of Object.entries(emojiMap)) {
      if (fruitType && fruitType.includes(key)) {
        return emoji;
      }
    }
    
    return '🍎'; // 默认苹果
  },

  /**
   * 根据健康分数获取健康等级
   * @param {number} healthScore - 健康分数
   * @returns {string} 健康等级
   */
  getHealthGrade: function(healthScore) {
    const score = Number(healthScore) || 0;
    if (score >= 90) return 'excellent';
    if (score >= 75) return 'good';
    if (score >= 60) return 'average';
    return 'poor';
  },

  /**
   * 根据综合分数获取总体评级
   * @param {number} overallScore - 综合分数
   * @returns {string} 总体评级
   */
  getOverallGrade: function(overallScore) {
    const score = Number(overallScore) || 0;
    if (score >= 90) return 'Excellent';
    if (score >= 75) return 'Good';
    if (score >= 60) return 'Average';
    return 'Poor';
  },

  /**
   * 处理检测错误
   * @param {Error} error - 错误对象
   */
  handleDetectionError: function(error) {
    console.error('AI检测失败:', error);
    
    // 清除超时定时器
    if (this.detectionTimeout) {
      clearTimeout(this.detectionTimeout);
      this.detectionTimeout = null;
    }
    
    this.setData({ detecting: false });
    
    // 显示错误提示
    let errorMessage = '检测失败，请重试';
    
    if (error.message) {
      if (error.message.includes('网络')) {
        errorMessage = '网络连接异常，请检查网络设置';
      } else if (error.message.includes('超时')) {
        errorMessage = 'AI服务响应超时，请稍后重试';
      } else if (error.message.includes('格式')) {
        errorMessage = '图片格式不支持，请重新选择';
      } else {
        errorMessage = error.message;
      }
    }
    
    wx.showModal({
      title: 'AI检测失败',
      content: errorMessage,
      confirmText: '重试',
      cancelText: '取消',
      success: (res) => {
        if (res.confirm) {
          // 延迟重试
          setTimeout(() => {
            this.startDetection();
          }, 1000);
        }
      }
    });
  },

  // ==================== 检测模式相关方法 ====================

  /**
   * 切换检测模式
   */
  toggleDetectionMode: function() {
    this.setData({ showModeModal: true });
  },

  /**
   * 隐藏模式选择模态框
   */
  hideModeModal: function() {
    this.setData({ showModeModal: false });
  },

  /**
   * 选择检测模式
   * @param {Object} e - 事件对象
   */
  selectDetectionMode: function(e) {
    const mode = e.currentTarget.dataset.mode;
    const modeText = this.data.detectionModeMap[mode];
    
    if (mode === this.data.detectionMode) {
      this.hideModeModal();
      return;
    }
    
    this.setData({
      detectionMode: mode,
      detectionModeText: modeText,
      showModeModal: false
    });
    
    // 保存到本地存储
    wx.setStorageSync('detection_mode', mode);
    
    // 显示切换成功提示
    wx.showToast({
      title: `已切换为${modeText}`,
      icon: 'none',
      duration: 1500
    });
    
    console.log('检测模式已切换:', mode);
  },

  // ==================== 历史记录相关方法 ====================

  /**
   * 保存检测记录到本地存储
   * @param {Object} result - 检测结果
   */
  saveDetectionRecord: function(result) {
    try {
      // 创建历史记录条目（确保所有字符串字段都安全）
      const historyItem = {
        id: this.safeString(result.id, generateDetectionId()),
        fruitType: this.safeString(result.fruitType, '未知水果'),
        maturity: result.maturity || 0,
        healthStatus: this.safeString(result.healthStatus, '未知'),
        qualityScore: result.qualityScore || 0,
        grade: this.safeString(result.overallGrade, 'Unknown'),
        detectionTime: this.formatTime(new Date()),
        location: '当前区域', // TODO: 从GPS或机器人位置获取
        actionTaken: '待处理',
        thumbnailUrl: result.imagePath,
        timestamp: result.timestamp
      };
      
      // 获取现有历史记录
      let history = wx.getStorageSync('detection_history') || [];
      
      // 添加新记录到开头
      history.unshift(historyItem);
      
      // 限制历史记录数量（最多保存100条）
      if (history.length > 100) {
        history = history.slice(0, 100);
      }
      
      // 保存到本地存储
      wx.setStorageSync('detection_history', history);
      
      // 更新页面数据
      this.setData({
        detectionHistory: history.slice(0, 20),
        hasMoreHistory: history.length > 20
      });
      
      // 重新计算统计数据
      this.calculateHistoryStats(history);
      
      console.log('检测记录已保存:', historyItem.id);
      
    } catch (error) {
      console.error('保存检测记录失败:', error);
    }
  },

  /**
   * 计算历史统计数据
   * @param {Array} history - 历史记录数组
   */
  calculateHistoryStats: function(history) {
    if (!history || history.length === 0) {
      this.setData({
        historyStats: {
          totalDetections: 0,
          excellentCount: 0,
          averageMaturity: 0,
          accuracyRate: 96 // 默认准确率
        }
      });
      return;
    }
    
    const stats = {
      totalDetections: history.length,
      excellentCount: 0,
      averageMaturity: 0,
      accuracyRate: 96 // 模拟准确率
    };
    
    let totalMaturity = 0;
    
    history.forEach(item => {
      // 统计优质果数量（Excellent等级）
      if (item.grade === 'Excellent') {
        stats.excellentCount++;
      }
      // 累加成熟度
      totalMaturity += item.maturity || 0;
    });
    
    // 计算平均成熟度
    stats.averageMaturity = Math.round(totalMaturity / history.length) || 0;
    
    this.setData({ historyStats: stats });
    
    console.log('历史统计计算完成:', stats);
  },

  /**
   * 日期筛选变化
   * @param {Object} e - 事件对象
   */
  onDateFilterChange: function(e) {
    const selectedDate = e.detail.value;
    const today = this.formatDate(new Date());
    
    let filterText = '今日';
    if (selectedDate !== today) {
      filterText = selectedDate;
    }
    
    this.setData({
      filterDate: selectedDate,
      filterDateText: filterText
    });
    
    // 根据日期筛选历史记录
    this.filterHistoryByDate(selectedDate);
  },

  /**
   * 根据日期筛选历史记录
   * @param {string} date - 筛选日期
   */
  filterHistoryByDate: function(date) {
    try {
      // 更新筛选日期
      this.setData({
        filterDate: date,
        filterDateText: date === this.formatDate(new Date()) ? '今日' : date
      });
      
      // 从服务器获取指定日期的历史记录
      this.requestDetectionHistoryFromServer();
      
      // 同时筛选本地历史记录作为备用
      const allHistory = wx.getStorageSync('detection_history') || [];
      
      let filteredHistory = allHistory;
      
      if (date) {
        filteredHistory = allHistory.filter(item => {
          const itemDate = this.formatDate(new Date(item.timestamp));
          return itemDate === date;
        });
      }
      
      this.setData({
        detectionHistory: filteredHistory.slice(0, 20),
        hasMoreHistory: filteredHistory.length > 20
      });
      
      // 重新计算统计数据
      this.calculateHistoryStats(filteredHistory);
      
      console.log('历史记录筛选完成:', {
        date: date,
        total: allHistory.length,
        filtered: filteredHistory.length
      });
      
    } catch (error) {
      console.error('筛选历史记录失败:', error);
    }
  },

  /**
   * 查看检测详情
   * @param {Object} e - 事件对象
   */
  viewDetectionDetail: function(e) {
    const detectionId = e.currentTarget.dataset.id;
    
    // TODO: 跳转到详情页面
    wx.navigateTo({
      url: `/pages/detection/detail?id=${detectionId}`,
      fail: () => {
        // 如果详情页面不存在，显示简单信息
        wx.showToast({
          title: '详情页面开发中',
          icon: 'none'
        });
      }
    });
  },

  /**
   * 快速操作
   * @param {Object} e - 事件对象
   */
  quickAction: function(e) {
    const action = e.currentTarget.dataset.action;
    const detectionId = e.currentTarget.dataset.id;
    
    switch (action) {
      case 'reanalyze':
        this.reanalyzeDetection(detectionId);
        break;
      case 'share':
        this.shareDetection(detectionId);
        break;
      default:
        console.log('未知操作:', action);
    }
  },

  /**
   * 重新分析检测
   * @param {string} detectionId - 检测ID
   */
  reanalyzeDetection: function(detectionId) {
    // TODO: 实现重新分析功能
    wx.showToast({
      title: '重新分析功能开发中',
      icon: 'none'
    });
  },

  /**
   * 分享检测结果
   * @param {string} detectionId - 检测ID
   */
  shareDetection: function(detectionId) {
    // TODO: 实现分享功能
    wx.showToast({
      title: '分享功能开发中',
      icon: 'none'
    });
  },

  /**
   * 加载更多历史记录
   */
  loadMoreHistory: function() {
    if (this.data.loadingMoreHistory) return;
    
    this.setData({ loadingMoreHistory: true });
    
    // 模拟加载延迟
    setTimeout(() => {
      try {
        const allHistory = wx.getStorageSync('detection_history') || [];
        let filteredHistory = allHistory;
        
        // 如果有日期筛选，应用筛选条件
        if (this.data.filterDate) {
          const today = this.formatDate(new Date());
          if (this.data.filterDate !== today) {
            filteredHistory = allHistory.filter(item => {
              const itemDate = this.formatDate(new Date(item.timestamp));
              return itemDate === this.data.filterDate;
            });
          }
        }
        
        const currentCount = this.data.detectionHistory.length;
        const nextBatch = filteredHistory.slice(currentCount, currentCount + 20);
        
        if (nextBatch.length > 0) {
          const newHistory = [...this.data.detectionHistory, ...nextBatch];
          
          this.setData({
            detectionHistory: newHistory,
            hasMoreHistory: newHistory.length < filteredHistory.length,
            loadingMoreHistory: false
          });
          
          // 不需要重新计算统计数据，因为这只是加载更多，不是重新筛选
        } else {
          this.setData({
            hasMoreHistory: false,
            loadingMoreHistory: false
          });
        }
        
      } catch (error) {
        console.error('加载更多历史记录失败:', error);
        this.setData({ loadingMoreHistory: false });
      }
    }, 1000);
  },

  // ==================== 结果操作相关方法 ====================

  /**
   * 分享检测结果
   */
  shareResult: function() {
    if (!this.data.detectionResult) return;
    
    const result = this.data.detectionResult;
    
    // 生成分享内容
    const shareContent = `🍎 AI水果检测结果
水果类型：${result.fruitType}
成熟度：${result.maturity}%
健康状况：${result.healthStatus}
品质评分：${result.qualityScore}/100
总体评级：${result.overallGrade}

建议：${result.recommendation}`;
    
    // 复制到剪贴板
    wx.setClipboardData({
      data: shareContent,
      success: () => {
        wx.showToast({
          title: '检测结果已复制到剪贴板',
          icon: 'success'
        });
      }
    });
  },

  /**
   * 保存检测结果
   */
  saveResult: function() {
    if (!this.data.detectionResult) return;
    
    // 检查保存到相册的权限
    wx.getSetting({
      success: (res) => {
        if (res.authSetting['scope.writePhotosAlbum'] === false) {
          // 用户之前拒绝了权限，引导用户去设置
          wx.showModal({
            title: '需要保存权限',
            content: '需要您授权保存图片到相册',
            confirmText: '去设置',
            success: (modalRes) => {
              if (modalRes.confirm) {
                wx.openSetting();
              }
            }
          });
        } else {
          // 尝试保存图片
          this.saveImageToAlbum();
        }
      }
    });
  },

  /**
   * 保存图片到相册的具体实现
   */
  saveImageToAlbum: function() {
    const that = this;
    
    wx.saveImageToPhotosAlbum({
      filePath: this.data.currentImage,
      success: () => {
        wx.showToast({
          title: '图片已保存到相册',
          icon: 'success'
        });
      },
      fail: (error) => {
        console.error('保存图片失败:', error);
        
        if (error.errMsg.includes('auth')) {
          // 权限被拒绝，请求权限
          wx.authorize({
            scope: 'scope.writePhotosAlbum',
            success: () => {
              // 重新尝试保存
              that.saveImageToAlbum();
            },
            fail: () => {
              wx.showToast({
                title: '保存失败，请检查相册权限',
                icon: 'none'
              });
            }
          });
        } else {
          wx.showToast({
            title: '保存失败，请重试',
            icon: 'none'
          });
        }
      }
    });
  },

  /**
   * 生成检测报告
   */
  generateReport: function() {
    if (!this.data.detectionResult) return;
    
    // TODO: 跳转到报告生成页面
    wx.navigateTo({
      url: '/pages/detection/report',
      fail: () => {
        wx.showToast({
          title: '报告功能开发中',
          icon: 'none'
        });
      }
    });
  },

  /**
   * 执行建议操作
   */
  executeSuggestedAction: function() {
    const result = this.data.detectionResult;
    if (!result || !result.actionable) return;
    
    const action = result.suggestedAction;
    
    wx.showModal({
      title: '执行建议操作',
      content: `确定要执行"${action === 'harvest' ? '立即采摘' : action === 'wait' ? '等待观察' : '标记检查'}"操作吗？`,
      success: (res) => {
        if (res.confirm) {
          // TODO: 与机器人控制系统集成
          this.sendActionCommand(action);
        }
      }
    });
  },

  /**
   * 设置提醒
   */
  setReminder: function() {
    // TODO: 实现提醒功能
    wx.showToast({
      title: '提醒功能开发中',
      icon: 'none'
    });
  },

  /**
   * 标记检查
   */
  markForInspection: function() {
    // TODO: 实现标记功能
    wx.showToast({
      title: '标记功能开发中',
      icon: 'none'
    });
  },

  // ==================== WebSocket消息处理 ====================

  /**
   * 处理WebSocket消息
   * @param {Object} data - 消息数据
   */
  onSocketMessage: function(data) {
    console.log('Detection页面收到WebSocket消息:', data.type);
    
    switch (data.type) {
      case 'ai_service_status':
        this.handleAIServiceStatus(data);
        break;
        
      case 'ai_fruit_detection_response':
        this.handleDetectionResponse(data);
        break;
        
      case 'ai_fruit_detection_error':
        this.handleDetectionError(new Error(data.message || '检测失败'));
        break;
        
      case 'fruit_detection_result':
        // 处理来自服务器的水果识别结果
        this.handleServerDetectionResult(data);
        break;
        
      case 'detection_history_response':
        // 处理来自服务器的历史记录响应
        this.handleDetectionHistoryResponse(data);
        break;
        
      default:
        console.log('未处理的消息类型:', data.type);
    }
  },

  /**
   * 从服务器请求水果识别历史记录
   */
  requestDetectionHistoryFromServer: function() {
    const app = getApp();
    
    if (!app.globalData || !app.globalData.connected) {
      console.log('WebSocket未连接，无法获取服务器历史记录');
      return;
    }
    
    try {
      app.sendSocketMessage({
        type: 'get_detection_history',
        robot_id: this.data.robotId,
        date_filter: this.data.filterDate,
        timestamp: Date.now()
      });
      
      console.log('已请求服务器历史记录');
      
    } catch (error) {
      console.error('请求服务器历史记录失败:', error);
    }
  },

  /**
   * 处理来自服务器的水果识别结果
   * @param {Object} data - 服务器发送的识别结果
   */
  handleServerDetectionResult: function(data) {
    console.log('收到服务器水果识别结果:', data);
    console.log('原始数据结构:', JSON.stringify(data, null, 2));
    
    try {
      // 格式化服务器返回的识别结果
      const result = this.formatServerDetectionResult(data);
      console.log('格式化后的结果:', JSON.stringify(result, null, 2));
      
      // 保存到本地历史记录
      this.saveDetectionRecord(result);
      
      // 更新统计数据
      this.updateTodayStats();
      
      // 显示通知
      wx.showToast({
        title: `收到新识别结果: ${result.fruitType}`,
        icon: 'success',
        duration: 2000
      });
      
      // 如果当前在检测页面，刷新显示
      this.refreshPageData();
      
    } catch (error) {
      console.error('处理服务器识别结果失败:', error);
      console.error('错误详情:', error.stack);
    }
  },

  /**
   * 格式化服务器返回的识别结果
   * @param {Object} serverData - 服务器数据
   * @returns {Object} 格式化后的结果
   */
  formatServerDetectionResult: function(serverData) {
    const detectionId = generateDetectionId();
    
    // 服务器数据字段映射（支持两种格式：直接字段名和嵌套data字段）
    const data = serverData.data || serverData;
    
    console.log('格式化数据 - 原始serverData:', serverData);
    console.log('格式化数据 - 提取的data:', data);
    console.log('格式化数据 - fruitType字段值:', data.fruitType);
    
    const result = {
      id: detectionId,
      fruitType: this.safeString(data.fruitType, '未知水果'),
      fruitEmoji: this.getFruitEmoji(data.fruitType),
      variety: this.safeString(data.variety, '未知品种'),
      confidence: Math.round(data.confidence || 0),
      maturity: Math.round(data.maturity || 0),
      healthStatus: this.safeString(data.healthStatus, '未知'),
      healthGrade: this.getHealthGrade(data.qualityScore),
      pestStatus: this.safeString(data.pestStatus, 'none'),
      diseaseStatus: this.safeString(data.diseaseStatus, 'none'),
      qualityScore: Math.round(data.qualityScore || 0),
      appearanceStars: Math.round((data.qualityScore || 0) / 20),
      sizeCategory: this.safeString(data.sizeCategory, '中等'),
      overallGrade: this.getOverallGrade(data.qualityScore),
      recommendation: this.safeString(data.recommendation, '暂无建议'),
      suggestedAction: this.safeString(data.suggestedAction, 'inspect'),
      actionable: data.actionable !== false,
      boundingBox: data.boundingBox || null,
      timestamp: data.timestamp || Date.now(),
      detectionTime: data.detectionTime || this.formatTime(new Date()),
      location: this.safeString(data.location, '未知位置'),
      actionTaken: this.safeString(data.actionTaken, '待处理'),
      imageName: data.source_image || '服务器图片',
      detectionMode: 'comprehensive',
      // 新增字段支持
      grade: this.safeString(data.grade, 'Average'),
      defects: Array.isArray(data.defects) ? data.defects : [],
      estimatedWeight: Math.round(data.estimatedWeight || 0),
      ripeness_days: data.ripeness_days || 0,
      marketValue: parseFloat(data.marketValue || 0),
      storageLife: Math.round(data.storageLife || 0),
      // 处理图片数据
      imageBase64: data.imageBase64 || '',  // 服务器传来的base64数据
      imageId: data.imageId || detectionId,  // 图片唯一标识
      imageFormat: data.imageFormat || 'jpg',  // 图片格式
      imageUrl: '',  // 初始为空，保存后会更新
      isLocalImage: true,  // 标记为本地图片
      needsSaveImage: !!data.imageBase64  // 是否需要保存图片
    };
    
    // 如果有base64图片数据，保存到本地
    if (result.needsSaveImage) {
      this.saveBase64ImageToLocal(result).then(savedPath => {
        console.log('图片保存成功:', savedPath);
        // 更新历史记录中的图片路径
        this.updateImagePathInHistory(result.id, savedPath);
      }).catch(error => {
        console.error('图片保存失败:', error);
        // 使用预设图片作为备选
        const fallbackPath = this.getLocalFruitImage(result.fruitType);
        this.updateImagePathInHistory(result.id, fallbackPath);
      });
    } else {
      // 没有图片数据，使用预设图片
      result.imageUrl = this.getLocalFruitImage(result.fruitType);
    }
    
    console.log('服务器图片处理:');
    console.log('- 水果类型:', data.fruitType);
    console.log('- 是否有base64数据:', !!data.imageBase64);
    console.log('- 图片ID:', result.imageId);
    console.log('- 格式化完成 - 最终结果fruitType:', result.fruitType);
    
    return result;
  },

  /**
   * 保存base64图片数据到本地
   * @param {Object} result - 识别结果对象
   * @returns {Promise<string>} 保存后的本地路径
   */
  saveBase64ImageToLocal: function(result) {
    return new Promise((resolve, reject) => {
      if (!result.imageBase64) {
        reject(new Error('没有图片数据'));
        return;
      }

      try {
        // 生成本地文件名
        const fileName = `${result.imageId}.${result.imageFormat}`;
        const tempFilePath = `${wx.env.USER_DATA_PATH}/${fileName}`;
        
        console.log('开始保存图片到本地:', fileName);
        
        // 将base64数据写入临时文件
        const fs = wx.getFileSystemManager();
        
        // 移除base64前缀（如果有）
        let base64Data = result.imageBase64;
        if (base64Data.startsWith('data:')) {
          base64Data = base64Data.split(',')[1];
        }
        
        // 写入文件
        fs.writeFile({
          filePath: tempFilePath,
          data: base64Data,
          encoding: 'base64',
          success: (res) => {
            console.log('图片写入成功:', tempFilePath);
            
            // 保存到永久存储
            wx.saveFile({
              tempFilePath: tempFilePath,
              success: (saveRes) => {
                console.log('图片保存到永久存储成功:', saveRes.savedFilePath);
                
                // 更新本地图片记录
                this.updateLocalImageRecord(fileName, saveRes.savedFilePath, result);
                
                resolve(saveRes.savedFilePath);
              },
              fail: (saveError) => {
                console.error('保存到永久存储失败:', saveError);
                // 即使永久保存失败，也可以使用临时路径
                resolve(tempFilePath);
              }
            });
          },
          fail: (writeError) => {
            console.error('图片写入失败:', writeError);
            reject(writeError);
          }
        });
        
      } catch (error) {
        console.error('保存图片过程中出错:', error);
        reject(error);
      }
    });
  },

  /**
   * 更新历史记录中的图片路径
   * @param {string} recordId - 记录ID
   * @param {string} imagePath - 图片路径
   */
  updateImagePathInHistory: function(recordId, imagePath) {
    try {
      // 更新页面数据
      const history = this.data.detectionHistory;
      const updatedHistory = history.map(item => {
        if (item.id === recordId) {
          item.imageUrl = imagePath;
          item.localImagePath = imagePath;
          console.log(`更新记录 ${recordId} 的图片路径:`, imagePath);
        }
        return item;
      });
      
      this.setData({
        detectionHistory: updatedHistory
      });
      
      // 更新本地存储
      const localHistory = wx.getStorageSync('detection_history') || [];
      const updatedLocalHistory = localHistory.map(item => {
        if (item.id === recordId) {
          item.imageUrl = imagePath;
          item.localImagePath = imagePath;
        }
        return item;
      });
      
      wx.setStorageSync('detection_history', updatedLocalHistory);
      
    } catch (error) {
      console.error('更新历史记录图片路径失败:', error);
    }
  },

  /**
   * 获取本地水果图片路径
   * @param {string} fruitType - 水果类型
   * @returns {string} 本地图片路径
   */
  getLocalFruitImage: function(fruitType) {
    // 首先尝试从预设图片库中获取
    if (this.data.fruitImageLibrary[fruitType]) {
      return this.data.fruitImageLibrary[fruitType];
    }
    
    // 尝试模糊匹配
    const fruitKeys = Object.keys(this.data.fruitImageLibrary);
    for (let key of fruitKeys) {
      if (key.includes(fruitType) || fruitType.includes(key)) {
        console.log(`模糊匹配水果图片: ${fruitType} -> ${key}`);
        return this.data.fruitImageLibrary[key];
      }
    }
    
    // 如果没有找到匹配的图片，返回默认图片
    console.log(`未找到匹配的水果图片: ${fruitType}，使用默认图片`);
    return '/images/fruits/default_fruit.jpg';
  },

  /**
   * 保存用户拍摄的图片到本地存储
   * @param {string} tempFilePath - 临时文件路径
   * @returns {Promise<string>} 保存后的本地路径
   */
  saveImageToLocal: function(tempFilePath) {
    return new Promise((resolve, reject) => {
      const timestamp = Date.now();
      const fileName = `fruit_detection_${timestamp}.jpg`;
      const savedFilePath = `${wx.env.USER_DATA_PATH}/${fileName}`;
      
      // 保存图片到本地
      wx.saveFile({
        tempFilePath: tempFilePath,
        success: (res) => {
          console.log('图片保存成功:', res.savedFilePath);
          
          // 更新本地图片记录
          this.updateLocalImageRecord(fileName, res.savedFilePath);
          
          resolve(res.savedFilePath);
        },
        fail: (error) => {
          console.error('图片保存失败:', error);
          reject(error);
        }
      });
    });
  },

  /**
   * 更新本地图片记录
   * @param {string} fileName - 文件名
   * @param {string} filePath - 文件路径
   * @param {Object} detectionResult - 识别结果（可选）
   */
  updateLocalImageRecord: function(fileName, filePath, detectionResult = null) {
    try {
      let localImages = wx.getStorageSync('local_fruit_images') || [];
      
      // 添加新记录
      const newRecord = {
        fileName: fileName,
        filePath: filePath,
        timestamp: Date.now(),
        size: 0,  // 可以后续获取文件大小
        fruitType: detectionResult ? detectionResult.fruitType : '未知',
        detectionId: detectionResult ? detectionResult.id : null
      };
      
      localImages.unshift(newRecord);
      
      // 限制最大数量
      if (localImages.length > this.data.maxLocalImages) {
        const removedImages = localImages.splice(this.data.maxLocalImages);
        // 删除超出限制的图片文件
        removedImages.forEach(img => {
          wx.removeSavedFile({
            filePath: img.filePath,
            complete: () => {
              console.log('删除旧图片:', img.fileName);
            }
          });
        });
      }
      
      // 保存更新后的记录
      wx.setStorageSync('local_fruit_images', localImages);
      
      console.log('本地图片记录已更新:', newRecord);
      
    } catch (error) {
      console.error('更新本地图片记录失败:', error);
    }
  },

  /**
   * 获取本地保存的图片列表
   * @returns {Array} 本地图片列表
   */
  getLocalImageList: function() {
    try {
      return wx.getStorageSync('local_fruit_images') || [];
    } catch (error) {
      console.error('获取本地图片列表失败:', error);
      return [];
    }
  },

  /**
   * 清理本地图片缓存
   */
  cleanupLocalImages: function() {
    try {
      const localImages = this.getLocalImageList();
      const now = Date.now();
      const maxAge = 7 * 24 * 60 * 60 * 1000; // 7天
      
      const validImages = [];
      const expiredImages = [];
      
      localImages.forEach(img => {
        if (now - img.timestamp > maxAge) {
          expiredImages.push(img);
        } else {
          validImages.push(img);
        }
      });
      
      // 删除过期图片
      expiredImages.forEach(img => {
        wx.removeSavedFile({
          filePath: img.filePath,
          complete: () => {
            console.log('清理过期图片:', img.fileName);
          }
        });
      });
      
      // 保存有效图片列表
      wx.setStorageSync('local_fruit_images', validImages);
      
      console.log(`图片清理完成: 删除 ${expiredImages.length} 张过期图片，保留 ${validImages.length} 张有效图片`);
      
    } catch (error) {
      console.error('清理本地图片失败:', error);
    }
  },

  /**
   * 处理来自服务器的历史记录响应
   * @param {Object} data - 服务器返回的历史记录数据
   */
  handleDetectionHistoryResponse: function(data) {
    console.log('收到服务器历史记录响应:', data);
    
    try {
      if (data.success && data.history && Array.isArray(data.history)) {
        // 格式化历史记录数据
        const formattedHistory = data.history.map(record => 
          this.formatServerDetectionResult(record)
        );
        
        // 合并本地和服务器历史记录
        const localHistory = wx.getStorageSync('detection_history') || [];
        const mergedHistory = this.mergeHistoryRecords(localHistory, formattedHistory);
        
        // 保存合并后的历史记录
        wx.setStorageSync('detection_history', mergedHistory);
        
        // 更新页面显示
        this.setData({
          detectionHistory: mergedHistory.slice(0, 20),
          hasMoreHistory: mergedHistory.length > 20
        });
        
        // 重新计算统计数据
        this.calculateHistoryStats(mergedHistory);
        
        console.log(`成功加载 ${formattedHistory.length} 条服务器历史记录`);
        
      } else {
        console.log('服务器返回空历史记录或请求失败');
      }
      
    } catch (error) {
      console.error('处理服务器历史记录响应失败:', error);
    }
  },

  /**
   * 合并本地和服务器历史记录，去重并按时间排序
   * @param {Array} localHistory - 本地历史记录
   * @param {Array} serverHistory - 服务器历史记录
   * @returns {Array} 合并后的历史记录
   */
  mergeHistoryRecords: function(localHistory, serverHistory) {
    // 创建一个Map来去重，以timestamp为key
    const recordMap = new Map();
    
    // 添加本地记录
    localHistory.forEach(record => {
      if (record.timestamp) {
        recordMap.set(record.timestamp, record);
      }
    });
    
    // 添加服务器记录（会覆盖相同timestamp的本地记录）
    serverHistory.forEach(record => {
      if (record.timestamp) {
        recordMap.set(record.timestamp, record);
      }
    });
    
    // 转换为数组并按时间倒序排列
    const mergedHistory = Array.from(recordMap.values())
      .sort((a, b) => (b.timestamp || 0) - (a.timestamp || 0));
    
    // 限制最大记录数量
    return mergedHistory.slice(0, 100);
  },

  // ==================== 图片处理相关方法 ====================

  /**
   * 图片加载错误处理
   */
  onImageError: function(e) {
    const itemId = e.currentTarget.dataset.id;
    const imgSrc = e.detail.errMsg;
    console.log(`图片加载失败，记录ID: ${itemId}, 图片URL: ${e.currentTarget.src}, 错误: ${imgSrc}`);
    console.log(`当前serverBaseUrl: ${this.data.serverBaseUrl}`);
    
    // 尝试使用备用URL
    const history = this.data.detectionHistory;
    const updatedHistory = history.map(item => {
      if (item.id === itemId) {
        // 设置加载失败标记
        item.imageLoadFailed = true;
        console.log(`设置图片加载失败标记，原始imageUrl: ${item.imageUrl}`);
      }
      return item;
    });
    
    this.setData({
      detectionHistory: updatedHistory
    });
    
    wx.showToast({
      title: '图片加载失败',
      icon: 'none',
      duration: 1000
    });
  },

  // ==================== 工具方法 ====================

  /**
   * 清除本地存储的检测数据（用于测试）
   */
  clearLocalDetectionData: function() {
    wx.showModal({
      title: '清除本地数据',
      content: '确定要清除所有本地检测历史记录吗？此操作不可恢复。',
      success: (res) => {
        if (res.confirm) {
          try {
            // 清除本地存储
            wx.removeStorageSync('detection_history');
            wx.removeStorageSync('today_detection_stats');
            
            // 重置页面数据
            this.setData({
              detectionHistory: [],
              todayDetectionCount: 0,
              hasMoreHistory: false,
              historyStats: {
                totalDetections: 0,
                excellentCount: 0,
                averageMaturity: 0,
                accuracyRate: 0
              }
            });
            
            wx.showToast({
              title: '本地数据已清除',
              icon: 'success'
            });
            
            // 重新从服务器获取数据
            setTimeout(() => {
              this.requestDetectionHistoryFromServer();
            }, 1000);
            
          } catch (error) {
            console.error('清除本地数据失败:', error);
            wx.showToast({
              title: '清除失败',
              icon: 'error'
            });
          }
        }
      }
    });
  },

  /**
   * 发送动作命令到机器人
   * @param {string} action - 动作类型
   */
  sendActionCommand: function(action) {
    const app = getApp();
    
    if (!app.globalData || !app.globalData.connected) {
      this.showErrorToast('连接已断开，无法执行操作');
      return;
    }
    
    try {
      app.sendSocketMessage({
        type: 'robot_action_command',
        action: action,
        robot_id: this.data.robotId,
        timestamp: Date.now()
      });
      
      wx.showToast({
        title: '指令已发送',
        icon: 'success'
      });
      
    } catch (error) {
      console.error('发送动作命令失败:', error);
      this.showErrorToast('指令发送失败');
    }
  },

  /**
   * 请求相机权限
   * @param {Function} callback - 授权成功回调
   */
  requestCameraPermission: function(callback) {
    wx.authorize({
      scope: 'scope.camera',
      success: () => {
        this.setData({ cameraAuthorized: true });
        if (callback) callback();
      },
      fail: () => {
        wx.showModal({
          title: '需要相机权限',
          content: '使用拍照功能需要开启相机权限，请前往设置页面开启',
          confirmText: '去设置',
          success: (res) => {
            if (res.confirm) {
              wx.openSetting({
                success: (settingRes) => {
                  if (settingRes.authSetting['scope.camera']) {
                    this.setData({ cameraAuthorized: true });
                    if (callback) callback();
                  }
                }
              });
            }
          }
        });
      }
    });
  },

  /**
   * 请求相册权限
   * @param {Function} callback - 授权成功回调
   */
  requestAlbumPermission: function(callback) {
    wx.authorize({
      scope: 'scope.album', // 修改为正确的权限名称
      success: () => {
        this.setData({ albumAuthorized: true });
        if (callback) callback();
      },
      fail: () => {
        wx.showModal({
          title: '需要相册权限',
          content: '使用相册选择功能需要开启相册权限，请前往设置页面开启',
          confirmText: '去设置',
          success: (res) => {
            if (res.confirm) {
              wx.openSetting({
                success: (settingRes) => {
                  if (settingRes.authSetting['scope.album']) {
                    this.setData({ albumAuthorized: true });
                    if (callback) callback();
                  }
                }
              });
            }
          }
        });
      }
    });
  },

  /**
   * 更新连接状态
   */
  updateConnectionStatus: function() {
    const app = getApp();
    const connected = app.globalData && app.globalData.connected;
    
    this.setData({ connected });
    
    if (!connected) {
      this.setData({ aiServiceStatus: 'offline' });
    } else {
      this.checkAIServiceStatus();
    }
    
    console.log('智能识别页面连接状态更新:', connected);
  },

  /**
   * 刷新页面数据
   */
  refreshPageData: function() {
    return new Promise((resolve) => {
      // 重新加载本地数据
      this.loadLocalData();
      
      // 更新连接状态
      this.updateConnectionStatus();
      
      // 刷新今日统计
      this.refreshTodayStats();
      
      // 检查AI服务状态
      this.checkAIServiceStatus();
      
      setTimeout(resolve, 1000);
    });
  },

  /**
   * 刷新今日统计数据
   */
  refreshTodayStats: function() {
    try {
      const todayStats = wx.getStorageSync('today_detection_stats') || {
        date: this.formatDate(new Date()),
        count: 0
      };
      
      const today = this.formatDate(new Date());
      if (todayStats.date !== today) {
        todayStats.date = today;
        todayStats.count = 0;
        wx.setStorageSync('today_detection_stats', todayStats);
      }
      
      this.setData({ todayDetectionCount: todayStats.count });
      
      // 如果当前筛选的是今日，更新统计数据
      if (this.data.filterDate === today || this.data.filterDateText === '今日') {
        this.filterHistoryByDate(today);
      }
      
    } catch (error) {
      console.error('刷新今日统计失败:', error);
    }
  },

  /**
   * 更新今日统计数据
   */
  updateTodayStats: function() {
    try {
      const today = this.formatDate(new Date());
      const todayStats = {
        date: today,
        count: this.data.todayDetectionCount + 1
      };
      
      wx.setStorageSync('today_detection_stats', todayStats);
      this.setData({ todayDetectionCount: todayStats.count });
      
    } catch (error) {
      console.error('更新今日统计失败:', error);
    }
  },

  /**
   * 清理定时器
   */
  clearTimers: function() {
    if (this.detectionTimeout) {
      clearTimeout(this.detectionTimeout);
      this.detectionTimeout = null;
    }
  },

  /**
   * 清理临时图片文件
   */
  clearTempImages: function() {
    // TODO: 实现临时文件清理
    console.log('清理临时图片文件');
  },

  /**
   * 格式化日期为 YYYY-MM-DD 格式
   * @param {Date} date - 日期对象
   * @returns {string} 格式化的日期字符串
   */
  formatDate: function(date) {
    const year = date.getFullYear();
    const month = (date.getMonth() + 1).toString().padStart(2, '0');
    const day = date.getDate().toString().padStart(2, '0');
    return `${year}-${month}-${day}`;
  },

  /**
   * 格式化时间为 HH:MM 格式
   * @param {Date} date - 日期对象
   * @returns {string} 格式化的时间字符串
   */
  formatTime: function(date) {
    const hours = date.getHours().toString().padStart(2, '0');
    const minutes = date.getMinutes().toString().padStart(2, '0');
    return `${hours}:${minutes}`;
  },

  /**
   * 显示错误提示
   * @param {string} message - 错误消息
   */
  showErrorToast: function(message) {
    wx.showToast({
      title: message,
      icon: 'none',
      duration: 2000
    });
  },

  // ==================== 安全字符串处理方法 ====================

  /**
   * 安全的toLowerCase转换
   * @param {any} str - 需要转换的值
   * @returns {string} 转换后的小写字符串，如果输入无效则返回空字符串
   */
  safeToLowerCase: function(str) {
    if (str === null || str === undefined) {
      return '';
    }
    return String(str).toLowerCase();
  },

  /**
   * 安全的charAt操作
   * @param {any} str - 字符串
   * @param {number} index - 索引位置
   * @returns {string} 指定位置的字符，如果无效则返回空字符串
   */
  safeCharAt: function(str, index = 0) {
    if (str === null || str === undefined) {
      return '';
    }
    const stringValue = String(str);
    return stringValue.charAt(index) || '';
  },

  /**
   * 安全的字符串获取
   * @param {any} value - 需要转换的值
   * @param {string} defaultValue - 默认值
   * @returns {string} 安全的字符串值
   */
  safeString: function(value, defaultValue = '') {
    if (value === null || value === undefined) {
      return defaultValue;
    }
    return String(value);
  },

  /**
   * 从详情页面返回时重新分析
   * @param {Object} detectionData - 检测数据
   */
  reanalyzeFromDetail: function(detectionData) {
    if (!detectionData || !detectionData.imageUrl) {
      wx.showToast({
        title: '无法重新分析，缺少图片信息',
        icon: 'none'
      });
      return;
    }

    // 设置当前图片
    this.setData({
      currentImage: detectionData.imageUrl,
      imageName: detectionData.imageName || '重新分析图片',
      imageTime: this.formatTime(new Date()),
      detectionResult: null
    });

    // 显示提示
    wx.showToast({
      title: '图片已加载，可以开始重新分析',
      icon: 'success'
    });

    // 自动滚动到检测区域
    setTimeout(() => {
      wx.pageScrollTo({
        selector: '.detection-card',
        duration: 300
      });
    }, 500);
  }
});