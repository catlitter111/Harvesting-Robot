// pages/settings/settings.js
Page({
    data: {
      // 开关设置
      autoSleep: true,
      autoIdentify: true,
      collisionDetection: true,
      statusNotification: true,
      taskNotification: true,
      alertNotification: true,
      
      // 采摘模式选择
      harvestModes: ['标准模式', '高效模式', '精细模式', '自定义模式'],
      harvestModeIndex: 0,
      
      // 其他设置
      harvestSpeed: 3,
      harvestThreshold: 85
    },
  
    onLoad: function (options) {
      // 加载设备设置信息
      this.loadDeviceSettings();
    },
    
    // 加载设备设置
    loadDeviceSettings: function() {
      // 这里应该是从服务器获取设备设置的代码
      // 现在只是使用本地数据
      console.log('加载设备设置');
    },
    
    // 切换自动休眠
    toggleAutoSleep: function(e) {
      this.setData({
        autoSleep: e.detail.value
      });
      this.updateSetting('autoSleep', e.detail.value);
    },
    
    // 切换自动识别
    toggleAutoIdentify: function(e) {
      this.setData({
        autoIdentify: e.detail.value
      });
      this.updateSetting('autoIdentify', e.detail.value);
    },
    
    // 切换碰撞监测
    toggleCollisionDetection: function(e) {
      this.setData({
        collisionDetection: e.detail.value
      });
      this.updateSetting('collisionDetection', e.detail.value);
    },
    
    // 切换状态通知
    toggleStatusNotification: function(e) {
      this.setData({
        statusNotification: e.detail.value
      });
      this.updateSetting('statusNotification', e.detail.value);
    },
    
    // 切换任务完成通知
    toggleTaskNotification: function(e) {
      this.setData({
        taskNotification: e.detail.value
      });
      this.updateSetting('taskNotification', e.detail.value);
    },
    
    // 切换异常警报
    toggleAlertNotification: function(e) {
      this.setData({
        alertNotification: e.detail.value
      });
      this.updateSetting('alertNotification', e.detail.value);
    },
    
    // 更改采摘模式
    changeHarvestMode: function(e) {
      this.setData({
        harvestModeIndex: e.detail.value
      });
      this.updateSetting('harvestMode', this.data.harvestModes[e.detail.value]);
      
      // 模式变更提示
      wx.showToast({
        title: `已切换为${this.data.harvestModes[e.detail.value]}`,
        icon: 'none'
      });
    },
    
    // 设置采摘速度
    setHarvestSpeed: function(e) {
      this.setData({
        harvestSpeed: e.detail.value
      });
      this.updateSetting('harvestSpeed', e.detail.value);
    },
    
    // 设置采摘阈值
    setHarvestThreshold: function() {
      const that = this;
      wx.showActionSheet({
        itemList: ['75%', '80%', '85%', '90%', '95%'],
        success(res) {
          const thresholds = [75, 80, 85, 90, 95];
          that.setData({
            harvestThreshold: thresholds[res.tapIndex]
          });
          that.updateSetting('harvestThreshold', thresholds[res.tapIndex]);
          
          wx.showToast({
            title: `采摘阈值已设为${thresholds[res.tapIndex]}%`,
            icon: 'none'
          });
        }
      });
    },
    
    // 更新设置到服务器
    updateSetting: function(key, value) {
      // 这里应该是将设置更新到服务器的代码
      // 现在只是模拟
      console.log('更新设置:', key, value);
      
      // 模拟设置更新成功
      wx.showToast({
        title: '设置已更新',
        icon: 'success',
        duration: 1000
      });
    },
    
    // 重置设备
    resetDevice: function() {
      const that = this;
      wx.showModal({
        title: '确认重置',
        content: '重置将恢复设备的出厂设置，所有自定义配置将丢失。是否继续？',
        confirmColor: '#FF9800',
        success(res) {
          if (res.confirm) {
            // 这里应该是实际重置设备的代码
            // 现在只是模拟
            wx.showLoading({
              title: '正在重置...',
            });
            
            setTimeout(function() {
              wx.hideLoading();
              wx.showToast({
                title: '设备已重置',
                icon: 'success'
              });
              
              // 重新加载默认设置
              that.loadDeviceSettings();
            }, 2000);
          }
        }
      });
    },
    
    // 退出登录
    logoutDevice: function() {
      wx.showModal({
        title: '确认退出',
        content: '退出登录后将无法远程控制设备，是否继续？',
        confirmColor: '#f44336',
        success(res) {
          if (res.confirm) {
            // 这里应该是实际退出登录的代码
            wx.showToast({
              title: '已退出登录',
              icon: 'success',
              success() {
                // 跳转到登录页面
                setTimeout(function() {
                  wx.redirectTo({
                    url: '/pages/login/login',
                  });
                }, 1500);
              }
            });
          }
        }
      });
    }
  });