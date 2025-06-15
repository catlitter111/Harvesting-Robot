// pages/detection/detail.js
// 水果检测详情页面

Page({
  /**
   * 页面的初始数据
   */
  data: {
    detectionData: null,
    detectionId: null,
    loading: true,
    error: null
  },

  /**
   * 生命周期函数--监听页面加载
   */
  onLoad: function (options) {
    const detectionId = options.id;
    if (detectionId) {
      this.setData({ detectionId });
      this.loadDetectionData(detectionId);
    } else {
      this.setData({
        error: '缺少检测ID参数',
        loading: false
      });
    }
  },

  /**
   * 加载检测数据
   * @param {string} detectionId - 检测ID
   */
  loadDetectionData: function(detectionId) {
    try {
      // 从本地存储获取检测历史
      const detectionHistory = wx.getStorageSync('detection_history') || [];
      
      console.log('详情页面 - 加载检测数据, ID:', detectionId);
      console.log('详情页面 - 历史记录总数:', detectionHistory.length);
      
      // 查找对应的检测记录
      const detectionData = detectionHistory.find(item => item.id === detectionId);
      
      if (detectionData) {
        console.log('详情页面 - 找到检测记录:', JSON.stringify(detectionData, null, 2));
        console.log('详情页面 - recommendation字段:', detectionData.recommendation);
        console.log('详情页面 - recommendation字段类型:', typeof detectionData.recommendation);
        
        this.setData({
          detectionData: detectionData,
          loading: false
        });
        
        // 设置页面标题
        wx.setNavigationBarTitle({
          title: `${detectionData.fruitType || '水果'}检测详情`
        });
      } else {
        console.log('详情页面 - 未找到检测记录, ID:', detectionId);
        console.log('详情页面 - 可用的ID列表:', detectionHistory.map(item => item.id));
        
        this.setData({
          error: '未找到对应的检测记录',
          loading: false
        });
      }
    } catch (error) {
      console.error('加载检测数据失败:', error);
      this.setData({
        error: '加载数据失败',
        loading: false
      });
    }
  },

  /**
   * 预览图片
   */
  previewImage: function() {
    if (this.data.detectionData && this.data.detectionData.imageUrl) {
      wx.previewImage({
        urls: [this.data.detectionData.imageUrl],
        current: this.data.detectionData.imageUrl
      });
    }
  },

  /**
   * 分享检测结果
   */
  shareResult: function() {
    const data = this.data.detectionData;
    if (!data) return;

    // 生成分享内容
    const shareContent = `🍎 AI水果检测详情报告

🔍 基本信息
水果类型：${data.fruitType || '未知'}
品种：${data.variety || '未知'}
检测时间：${data.detectionTime || '未知'}
检测位置：${data.location || '未知'}

🌱 成熟度分析
成熟度：${data.maturity || 0}%
成熟阶段：${this.getMaturityStage(data.maturity)}
${data.ripeness_days !== undefined ? `采摘时机：${this.getRipenessText(data.ripeness_days)}` : ''}

🏥 健康状况
健康状态：${data.healthStatus || '未知'}
${data.defects && data.defects.length > 0 ? `发现缺陷：${data.defects.join('、')}` : '未发现明显缺陷'}

⭐ 品质评估
品质评分：${data.qualityScore || 0}/100
品质等级：${this.getQualityGrade(data.qualityScore)}
大小规格：${data.sizeCategory || '中等'}

${data.marketValue ? `💰 市场价值
市场价格：¥${data.marketValue}/斤` : ''}
${data.estimatedWeight ? `预估重量：${data.estimatedWeight}克` : ''}
${data.storageLife ? `储存期限：${data.storageLife}天` : ''}

💡 AI建议
${data.recommendation || '暂无建议'}

---
由智能水果识别系统生成`;

    // 复制到剪贴板
    wx.setClipboardData({
      data: shareContent,
      success: () => {
        wx.showToast({
          title: '检测详情已复制到剪贴板',
          icon: 'success',
          duration: 2000
        });
      },
      fail: () => {
        wx.showToast({
          title: '复制失败',
          icon: 'none'
        });
      }
    });
  },

  /**
   * 保存图片到相册
   */
  saveToAlbum: function() {
    const data = this.data.detectionData;
    if (!data || !data.imageUrl) {
      wx.showToast({
        title: '没有可保存的图片',
        icon: 'none'
      });
      return;
    }

    // 检查保存权限
    wx.getSetting({
      success: (res) => {
        if (res.authSetting['scope.writePhotosAlbum'] === false) {
          // 用户之前拒绝了权限
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
          this.saveImageToAlbum();
        }
      }
    });
  },

  /**
   * 保存图片到相册的具体实现
   */
  saveImageToAlbum: function() {
    const imageUrl = this.data.detectionData.imageUrl;
    
    wx.saveImageToPhotosAlbum({
      filePath: imageUrl,
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
              this.saveImageToAlbum();
            },
            fail: () => {
              wx.showToast({
                title: '保存失败，请授权相册权限',
                icon: 'none'
              });
            }
          });
        } else {
          wx.showToast({
            title: '保存失败',
            icon: 'none'
          });
        }
      }
    });
  },

  /**
   * 重新分析
   */
  reanalyze: function() {
    const data = this.data.detectionData;
    if (!data) return;

    wx.showModal({
      title: '重新分析',
      content: '确定要重新分析这张图片吗？',
      success: (res) => {
        if (res.confirm) {
          // 跳转回检测页面并传递图片信息
          wx.navigateBack({
            success: () => {
              // 通过事件通知上一页面进行重新分析
              const pages = getCurrentPages();
              if (pages.length > 0) {
                const prevPage = pages[pages.length - 1];
                if (prevPage.reanalyzeFromDetail) {
                  prevPage.reanalyzeFromDetail(data);
                }
              }
            }
          });
        }
      }
    });
  },

  /**
   * 获取成熟度阶段描述
   * @param {number} maturity - 成熟度百分比
   * @returns {string} 成熟度阶段
   */
  getMaturityStage: function(maturity) {
    if (!maturity) return '未知';
    
    if (maturity <= 20) return '幼果期';
    if (maturity <= 40) return '生长期';
    if (maturity <= 60) return '转色期';
    if (maturity <= 80) return '近熟期';
    if (maturity <= 95) return '最佳采摘期';
    return '过熟期';
  },

  /**
   * 获取采摘时机文本
   * @param {number} ripeness_days - 采摘天数
   * @returns {string} 采摘时机描述
   */
  getRipenessText: function(ripeness_days) {
    if (ripeness_days === undefined || ripeness_days === null) return '未知';
    
    if (ripeness_days === 0) return '立即采摘';
    if (ripeness_days > 0) return `${ripeness_days}天后采摘`;
    return `已过最佳期${Math.abs(ripeness_days)}天`;
  },

  /**
   * 获取成熟度建议
   * @param {number} maturity - 成熟度
   * @param {number} ripeness_days - 采摘天数
   * @returns {string} 建议文本
   */
  getMaturityRecommendation: function(maturity, ripeness_days) {
    if (!maturity) return '无法提供建议，缺少成熟度数据';
    
    if (maturity >= 80 && maturity <= 95) {
      return '水果已达到最佳成熟度，建议立即采摘以获得最佳口感和营养价值。';
    } else if (maturity > 95) {
      return '水果已过度成熟，建议尽快采摘，适合立即食用或加工处理。';
    } else if (maturity >= 60) {
      return '水果接近成熟，建议继续观察，预计几天内可达到最佳采摘期。';
    } else if (maturity >= 40) {
      return '水果正在转色期，需要继续等待成熟，建议一周后重新检测。';
    } else {
      return '水果尚未成熟，需要较长时间继续生长，建议2-3周后重新检测。';
    }
  },

  /**
   * 获取健康等级
   * @param {string} healthStatus - 健康状态
   * @returns {string} 健康等级类名
   */
  getHealthLevel: function(healthStatus) {
    if (!healthStatus) return 'unknown';
    
    const status = healthStatus.toLowerCase();
    if (status.includes('完全健康') || status.includes('excellent')) return 'excellent';
    if (status.includes('轻微') || status.includes('good')) return 'good';
    if (status.includes('中度') || status.includes('moderate')) return 'warning';
    if (status.includes('严重') || status.includes('poor')) return 'poor';
    return 'good';
  },

  /**
   * 获取健康图标
   * @param {string} healthStatus - 健康状态
   * @returns {string} 健康图标
   */
  getHealthIcon: function(healthStatus) {
    if (!healthStatus) return '❓';
    
    const status = healthStatus.toLowerCase();
    if (status.includes('完全健康') || status.includes('excellent')) return '💚';
    if (status.includes('轻微') || status.includes('good')) return '💛';
    if (status.includes('中度') || status.includes('moderate')) return '🧡';
    if (status.includes('严重') || status.includes('poor')) return '❤️';
    return '💚';
  },

  /**
   * 获取品质等级
   * @param {number} qualityScore - 品质分数
   * @returns {string} 品质等级
   */
  getQualityGrade: function(qualityScore) {
    if (!qualityScore) return '未评级';
    
    if (qualityScore >= 90) return '优质';
    if (qualityScore >= 80) return '良好';
    if (qualityScore >= 70) return '一般';
    if (qualityScore >= 60) return '合格';
    return '不合格';
  },

  /**
   * 获取星级数量
   * @param {number} qualityScore - 品质分数
   * @returns {number} 星级数量
   */
  getStarCount: function(qualityScore) {
    if (!qualityScore) return 0;
    return Math.round(qualityScore / 20);
  },

  /**
   * 获取操作建议文本
   * @param {string} actionCode - 操作代码
   * @returns {string} 操作文本
   */
  getActionText: function(actionCode) {
    const actionMap = {
      'harvest_now': '立即采摘',
      'harvest_priority': '优先采摘',
      'harvest_normal': '正常采摘',
      'wait_3_days': '等待3天',
      'wait_week': '等待一周',
      'inspect_closely': '密切观察',
      'reject': '不建议采摘'
    };
    
    return actionMap[actionCode] || actionCode || '未知操作';
  },

  /**
   * 计算单果价值
   * @param {number} marketValue - 市场价格(元/斤)
   * @param {number} estimatedWeight - 预估重量(克)
   * @returns {string} 单果价值
   */
  calculateFruitValue: function(marketValue, estimatedWeight) {
    if (!marketValue || !estimatedWeight) return '0.00';
    
    // 转换为元/克，然后计算单果价值
    const pricePerGram = marketValue / 500; // 1斤 = 500克
    const fruitValue = pricePerGram * estimatedWeight;
    
    return fruitValue.toFixed(2);
  },

  /**
   * 生命周期函数--监听页面初次渲染完成
   */
  onReady: function () {
    // 页面渲染完成
  },

  /**
   * 生命周期函数--监听页面显示
   */
  onShow: function () {
    // 页面显示
  },

  /**
   * 生命周期函数--监听页面隐藏
   */
  onHide: function () {
    // 页面隐藏
  },

  /**
   * 生命周期函数--监听页面卸载
   */
  onUnload: function () {
    // 页面卸载
  },

  /**
   * 页面相关事件处理函数--监听用户下拉动作
   */
  onPullDownRefresh: function () {
    // 重新加载数据
    if (this.data.detectionId) {
      this.loadDetectionData(this.data.detectionId);
    }
    wx.stopPullDownRefresh();
  },

  /**
   * 页面上拉触底事件的处理函数
   */
  onReachBottom: function () {
    // 详情页面不需要处理上拉触底
  },

  /**
   * 用户点击右上角分享
   */
  onShareAppMessage: function () {
    const data = this.data.detectionData;
    return {
      title: data ? `${data.fruitType}检测详情` : '水果检测详情',
      path: `/pages/detection/detail?id=${this.data.detectionId}`,
      imageUrl: data ? data.imageUrl : ''
    };
  }
}); 