// app.js
App({
    globalData: {
      userInfo: null,
      socketTask: null,
      connected: false,
      connecting: false,
      serverUrl: "ws://101.201.150.96:1234/ws/wechat/",
      clientId: '',
      robotId: 'robot_123',
      statisticsPage: null,
      controlPage: null,
      lastPongTime: 0,
      reconnectDelay: 5000, // 初始重连延迟时间(毫秒)
      maxReconnectDelay: 30000, // 最大重连延迟时间(毫秒)
      reconnectAttempts: 0, // 重连尝试次数
      heartbeatTimer: null, // 心跳定时器
      reconnectTimer: null, // 重连定时器
      closedByUser: false, // 是否是用户主动关闭
      lastMessageTime: 0, // 最后收到消息的时间
      clientVersion: '1.1.0', // 客户端版本号，用于兼容性检查
      networkLatency: 0 // 网络延迟
    },
    
    onLaunch: function () {
      // 生成唯一客户端ID
      this.globalData.clientId = `user_${Date.now()}_${Math.floor(Math.random() * 10000)}`;
      
      // 连接WebSocket
      this.connectWebSocket();
      
      // 获取地理位置权限
      this.getLocationPermission();
    },
    
    // 获取地理位置权限
    getLocationPermission: function() {
      wx.getSetting({
        success: (res) => {
          if (!res.authSetting['scope.userLocation']) {
            wx.authorize({
              scope: 'scope.userLocation',
              success: () => {
                console.log('位置权限获取成功');
              },
              fail: (error) => {
                console.error('位置权限获取失败', error);
                wx.showModal({
                  title: '提示',
                  content: '需要您的位置权限来显示机器人位置信息',
                  confirmText: '去设置',
                  success: (res) => {
                    if (res.confirm) {
                      wx.openSetting();
                    }
                  }
                });
              }
            });
          }
        }
      });
    },
    
    // 连接WebSocket
    connectWebSocket: function() {
      const that = this;
      
      // 清除可能已存在的重连定时器
      if (this.globalData.reconnectTimer) {
        clearTimeout(this.globalData.reconnectTimer);
        this.globalData.reconnectTimer = null;
      }
      
      if (this.globalData.connecting || this.globalData.connected) {
        return;
      }
      
      this.globalData.connecting = true;
      
      // WebSocket URL
      const wsUrl = `${this.globalData.serverUrl}${this.globalData.clientId}`;
      
      console.log('正在连接WebSocket...', wsUrl);
      
      // 创建WebSocket连接
      this.globalData.socketTask = wx.connectSocket({
        url: wsUrl,
        success: function() {
          console.log('WebSocket连接请求已发送');
        },
        fail: function(error) {
          console.error('WebSocket连接失败', error);
          that.globalData.connecting = false;
          
          // 设置指数退避重连
          that.scheduleReconnect();
        }
      });
      
      // 监听WebSocket连接打开
      this.globalData.socketTask.onOpen(function() {
        console.log('WebSocket连接已打开');
        that.globalData.connected = true;
        that.globalData.connecting = false;
        that.globalData.reconnectAttempts = 0; // 重置重连尝试次数
        that.globalData.lastPongTime = Date.now(); // 初始化最后心跳响应时间
        
        // 发送初始化消息
        that.globalData.socketTask.send({
          data: JSON.stringify({
            type: 'init',
            robot_id: that.globalData.robotId,
            client_version: that.globalData.clientVersion,
            connection_time: Date.now()
          }),
          success: function() {
            console.log('初始化消息发送成功');
          },
          fail: function(error) {
            console.error('初始化消息发送失败', error);
          }
        });
        
        // 启动心跳检测
        that.startHeartbeat();
      });
      
      // 监听WebSocket接收消息
      this.globalData.socketTask.onMessage(function(res) {
        try {
          const data = JSON.parse(res.data);
          const now = Date.now();
          that.globalData.lastMessageTime = now; // 更新最后收到消息的时间
          
          // 处理心跳响应
          if (data.type === 'pong') {
            that.globalData.lastPongTime = now;
            // 计算网络延迟
            if (data.echo_timestamp) {
              that.globalData.networkLatency = now - data.echo_timestamp;
            }
            return;
          }
          
          // 处理机器人连接状态消息
          if (data.type === 'robot_connection_status') {
            if (that.globalData.controlPage) {
              that.globalData.controlPage.handleRobotConnectionStatus(data);
            }
            return;
          }
          
          // 根据消息类型处理
          switch (data.type) {
            case 'statistics_update':
              // 如果统计页面存在，则转发消息
              if (that.globalData.statisticsPage) {
                that.globalData.statisticsPage.onSocketMessage(data);
              }
              break;
              
            case 'position_update':
              // 如果统计页面存在，则转发位置更新消息
              if (that.globalData.statisticsPage) {
                that.globalData.statisticsPage.onSocketMessage(data);
              }
              break;
              
            case 'position_history':
              // 如果统计页面存在，则转发位置历史数据消息
              if (that.globalData.statisticsPage) {
                that.globalData.statisticsPage.onSocketMessage(data);
              }
              break;
  
            case 'route_history':
              // 处理路线历史数据
              if (that.globalData.statisticsPage) {
                that.globalData.statisticsPage.onSocketMessage(data);
              }
              break;
              
            case 'video_frame':
              // 如果控制页面存在，则转发消息
              if (that.globalData.controlPage) {
                that.globalData.controlPage.handleVideoFrame(data);
              }
              break;
              
            case 'video_quality_update':
              // 如果控制页面存在，则转发消息
              if (that.globalData.controlPage) {
                that.globalData.controlPage.handleQualityUpdate(data);
              }
              break;
              
            case 'status_update':
              // 如果控制页面存在，则转发消息
              if (that.globalData.controlPage) {
                that.globalData.controlPage.updateRobotStatus(data.data);
              }
              break;
              
            case 'init_response':
              // 处理初始化响应
              if (data.success) {
                console.log('成功连接到机器人');
                
                // 请求最新统计数据
                if (that.globalData.statisticsPage) {
                  that.globalData.statisticsPage.requestStatisticsData();
                }
                
                // 通知控制页面机器人已连接
                if (that.globalData.controlPage && data.robot_online) {
                  that.globalData.controlPage.handleRobotConnectionStatus({
                    connected: true, 
                    timestamp: now
                  });
                }
              } else {
                console.warn('机器人未在线:', data.message);
                
                // 通知控制页面机器人离线
                if (that.globalData.controlPage) {
                  that.globalData.controlPage.handleRobotConnectionStatus({
                    connected: false, 
                    timestamp: now
                  });
                }
              }
              break;
              
            case 'video_stream_request_sent':
              // 通知控制页面视频流请求已发送
              console.log('视频流请求已发送到机器人');
              break;
              
            case 'ai_chat_response':
              // 如果聊天页面存在，则转发AI回复消息
              if (that.globalData.chatPage) {
                that.globalData.chatPage.onSocketMessage(data);
              }
              break;
              
            default:
              console.log('未知消息类型', data);
          }
        } catch (error) {
          console.error('解析WebSocket消息失败:', error);
        }
      });
      
      // 监听WebSocket错误
      this.globalData.socketTask.onError(function(error) {
        console.error('WebSocket发生错误:', error);
        that.globalData.connected = false;
        that.globalData.connecting = false;
        
        // 停止心跳
        that.stopHeartbeat();
        
        // 设置重连
        that.scheduleReconnect();
      });
      
      // 监听WebSocket关闭
      this.globalData.socketTask.onClose(function(event) {
        console.log('WebSocket已关闭, 代码:', event.code, '原因:', event.reason);
        that.globalData.connected = false;
        that.globalData.connecting = false;
        
        // 停止心跳
        that.stopHeartbeat();
        
        // 如果不是用户主动关闭，则尝试重连
        if (!that.globalData.closedByUser) {
          that.scheduleReconnect();
        }
      });
    },
    
    // 使用指数退避策略安排重连
    scheduleReconnect: function() {
      const that = this;
      
      // 清除可能存在的重连定时器
      if (this.globalData.reconnectTimer) {
        clearTimeout(this.globalData.reconnectTimer);
      }
      
      // 计算重连延迟
      const attempts = this.globalData.reconnectAttempts;
      // 使用指数退避策略，随着尝试次数增加，延迟时间增加
      const delay = Math.min(
        this.globalData.maxReconnectDelay,
        this.globalData.reconnectDelay * Math.pow(1.5, attempts)
      );
      
      console.log(`安排第 ${attempts + 1} 次重连，延迟 ${delay}ms`);
      
      // 设置定时器
      this.globalData.reconnectTimer = setTimeout(function() {
        // 增加重连尝试次数
        that.globalData.reconnectAttempts++;
        // 尝试重连
        that.connectWebSocket();
      }, delay);
    },
    
    // 启动心跳机制，保持连接活跃
    startHeartbeat: function() {
      const that = this;
      
      // 清除可能存在的旧定时器
      if (this.globalData.heartbeatTimer) {
        clearInterval(this.globalData.heartbeatTimer);
      }
      
      this.globalData.lastPongTime = Date.now();
      
      // 每15秒发送一次心跳
      this.globalData.heartbeatTimer = setInterval(() => {
        if (that.globalData.connected) {
          // 检查上次pong响应时间，如果超过30秒没收到响应，认为连接已断开
          const now = Date.now();
          if (now - that.globalData.lastPongTime > 30000) {
            console.warn('心跳超时，连接可能已断开');
            that.globalData.connected = false;
            
            // 尝试关闭连接
            if (that.globalData.socketTask) {
              try {
                that.globalData.socketTask.close({
                  code: 1000,
                  reason: '心跳超时'
                });
              } catch (e) {
                console.error('关闭超时连接失败:', e);
              }
            }
            
            // 停止心跳
            that.stopHeartbeat();
            
            // 重新连接
            that.scheduleReconnect();
            
            return;
          }
          
          // 发送心跳消息
          that.sendSocketMessage({
            type: 'ping',
            timestamp: now
          });
        }
      }, 15000);
    },
    
    // 停止心跳
    stopHeartbeat: function() {
      if (this.globalData.heartbeatTimer) {
        clearInterval(this.globalData.heartbeatTimer);
        this.globalData.heartbeatTimer = null;
      }
    },
    
    // 主动关闭连接
    closeConnection: function() {
      const that = this;
      
      // 标记为用户主动关闭
      this.globalData.closedByUser = true;
      
      // 停止心跳
      this.stopHeartbeat();
      
      // 清除重连定时器
      if (this.globalData.reconnectTimer) {
        clearTimeout(this.globalData.reconnectTimer);
        this.globalData.reconnectTimer = null;
      }
      
      // 关闭连接
      if (this.globalData.socketTask && this.globalData.connected) {
        this.globalData.socketTask.close({
          code: 1000,
          reason: '用户主动关闭'
        });
        this.globalData.connected = false;
      }
      
      console.log('WebSocket连接已主动关闭');
      
      // 3秒后重置状态，允许再次连接
      setTimeout(() => {
        that.globalData.closedByUser = false;
      }, 3000);
    },
    
    // 发送WebSocket消息的通用方法
    sendSocketMessage: function(msg) {
      if (this.globalData.socketTask && this.globalData.connected) {
        // 添加发送时间戳
        if (typeof msg === 'object') {
          msg.client_timestamp = Date.now();
        }
        
        this.globalData.socketTask.send({
          data: JSON.stringify(msg),
          success: function() {
            // 成功发送无需处理
          },
          fail: function(error) {
            console.error('消息发送失败:', error);
            
            // 可能是连接已断开，尝试重连
            if (!this.globalData.connected && !this.globalData.connecting) {
              this.connectWebSocket();
            }
          }
        });
      } else {
        console.warn('WebSocket未连接或正在连接中，无法发送消息');
        
        // 尝试重新连接
        if (!this.globalData.connected && !this.globalData.connecting) {
          this.connectWebSocket();
        }
      }
    }
  });