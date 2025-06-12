// 聊天页面
const util = require('../../utils/util.js');

// 全局消息ID计数器，确保ID唯一性
let messageIdCounter = 0;

// 生成唯一消息ID的函数
function generateUniqueMessageId(type) {
  messageIdCounter++;
  const timestamp = Date.now();
  const random = Math.random().toString(36).substr(2, 9);
  const counter = messageIdCounter.toString().padStart(5, '0');
  return `${type}_${timestamp}_${counter}_${random}`;
}

Page({
  data: {
    messages: [
      // 示例消息用于UI展示
      {
        id: generateUniqueMessageId('welcome'),
        type: 'ai',
        text: '您好！我是AgriSage智能助手，很高兴为您服务。有什么问题可以随时问我！',
        time: '10:30',
        status: 'success',
        expanded: false
      }
    ],
    inputText: '',
    sending: false,
    scrollTop: 0,
    connected: false,
    scrollIntoView: '',
    messageQueue: [], // 消息渲染队列
    lastSendTime: 0, // 上次发送消息时间（防止频繁发送）
    sendingCount: 0, // 连续发送计数
    networkRetryCount: 0, // 网络重试次数
    maxRetryCount: 3 // 最大重试次数
  },

  // 页面实例属性（不存储在data中）
  pendingMessages: null, // 待响应消息映射 {timestamp: messageId}
  messageTimeouts: null, // 消息超时计时器
  connectionCheckInterval: null, // 连接检查定时器

  onLoad: function(options) {
    console.log('Chat页面加载');
    this.initializePage();
  },

  onShow: function() {
    console.log('Chat页面显示');
    this.resumePage();
  },

  onHide: function() {
    console.log('Chat页面隐藏');
    // 页面隐藏时保存状态（可选）
  },

  onUnload: function() {
    console.log('Chat页面卸载');
    
    // 清理所有待响应消息和计时器
    this.clearAllPendingMessages();
    
    // 停止网络监控
    this.stopNetworkMonitoring();
    
    // 取消注册
    const app = getApp();
    app.globalData.chatPage = null;
  },

  // 初始化页面
  initializePage: function() {
    // 初始化Map实例
    this.pendingMessages = new Map();
    this.messageTimeouts = new Map();
    
    // 注册页面到全局，以便接收WebSocket消息
    const app = getApp();
    app.globalData.chatPage = this;
    
    // 加载消息历史
    const hasHistory = this.loadMessageHistory();
    
    // 检查WebSocket连接状态
    this.checkConnection();
    
    // 启动网络状态监控
    this.startNetworkMonitoring();
    
    // 初始化滚动位置
    setTimeout(() => {
      this.scrollToBottom();
    }, 300); // 延迟一下确保数据渲染完成
    
    console.log('Chat页面初始化完成', hasHistory ? '(含历史消息)' : '(仅默认消息)');
  },

  // 页面恢复显示
  resumePage: function() {
    // 重新注册页面（防止被其他页面覆盖）
    const app = getApp();
    app.globalData.chatPage = this;
    
    // 检查连接状态
    this.checkConnection();
    
    // 滚动到底部显示最新消息
    this.scrollToBottom();
    
    // 如果有待发送的消息，恢复发送状态
    if (this.data.sending) {
      console.log('页面恢复时检测到发送中状态');
    }
  },

  // 检查WebSocket连接状态
  checkConnection: function() {
    const app = getApp();
    const connected = app.globalData && app.globalData.connected;
    
    this.setData({
      connected: connected
    });
    
    if (!connected) {
      console.warn('WebSocket连接已断开');
    }
  },

  // 输入验证
  validateInput: function(inputText) {
    // 空消息检查
    if (!inputText) {
      return { valid: false, message: '请输入消息内容' };
    }

    // 正在发送检查
    if (this.data.sending) {
      return { valid: false, message: '消息发送中，请稍候' };
    }

    // 长度限制检查
    if (inputText.length > 500) {
      return { valid: false, message: '消息内容过长，请控制在500字以内' };
    }

    // 连续空格或换行检查
    if (/^\s+$/.test(inputText)) {
      return { valid: false, message: '消息内容不能只包含空格' };
    }

    // 特殊字符过滤
    const filteredText = this.filterSpecialChars(inputText);
    if (filteredText !== inputText) {
      this.setData({ inputText: filteredText });
      return { valid: false, message: '消息包含特殊字符已自动过滤，请重新发送' };
    }

    // 敏感词检查（简单实现）
    const sensitiveWords = ['测试敏感词', '危险词汇']; // 实际项目中应该从服务器获取
    for (let word of sensitiveWords) {
      if (inputText.includes(word)) {
        return { valid: false, message: '消息包含敏感词汇，请修改后发送' };
      }
    }

    return { valid: true };
  },

  // 过滤特殊字符
  filterSpecialChars: function(text) {
    // 移除或替换危险字符，保留中文、英文、数字、常见标点
    return text.replace(/[<>\"'&\x00-\x08\x0B\x0C\x0E-\x1F\x7F]/g, '');
  },

  // 检查网络连接
  checkNetworkConnection: function() {
    const app = getApp();
    if (!app.globalData || !app.globalData.connected) {
      this.setData({ networkRetryCount: 0 }); // 重置重试计数
      wx.showModal({
        title: '网络连接异常',
        content: '网络连接已断开，请检查网络后重试',
        showCancel: true,
        cancelText: '取消',
        confirmText: '重试连接',
        success: (res) => {
          if (res.confirm) {
            this.retryConnection();
          }
        }
      });
      return false;
    }
    return true;
  },

  // 检查发送频率
  checkSendFrequency: function() {
    const now = Date.now();
    const timeDiff = now - this.data.lastSendTime;
    
    // 防止过于频繁发送（2秒内限制）
    if (timeDiff < 2000) {
      wx.showToast({
        title: '发送过于频繁，请稍后再试',
        icon: 'none',
        duration: 2000
      });
      return false;
    }

    // 连续发送计数检查
    if (this.data.sendingCount >= 5) {
      wx.showToast({
        title: '发送次数过多，请稍作休息',
        icon: 'none',
        duration: 3000
      });
      return false;
    }

    // 更新发送时间和计数
    this.setData({
      lastSendTime: now,
      sendingCount: this.data.sendingCount + 1
    });

    // 10秒后重置发送计数
    setTimeout(() => {
      this.setData({
        sendingCount: Math.max(0, this.data.sendingCount - 1)
      });
    }, 10000);

    return true;
  },

  // 发送消息
  sendMessage: function() {
    const inputText = this.data.inputText.trim();
    
    // 基础验证
    const validationResult = this.validateInput(inputText);
    if (!validationResult.valid) {
      wx.showToast({
        title: validationResult.message,
        icon: 'none',
        duration: 2000
      });
      return;
    }

    // 检查WebSocket连接
    if (!this.checkNetworkConnection()) {
      return;
    }

    // 频率限制检查
    if (!this.checkSendFrequency()) {
      return;
    }

    // 生成时间戳 - 确保使用整数时间戳
    const timestamp = Math.floor(Date.now());
    
    // 创建用户消息
    const userMessage = {
      id: generateUniqueMessageId('user'),
      type: 'user',
      text: inputText,
      time: this.formatTime(new Date()),
      status: 'sending',
      expanded: false,
      timestamp: timestamp  // 添加timestamp到消息对象
    };

    // 添加消息到列表
    this.addMessage(userMessage);
    this.setData({
      inputText: '',
      sending: true
    });

    // 记录待响应消息 - 使用字符串key确保一致性
    const timestampKey = timestamp.toString();
    this.pendingMessages.set(timestampKey, userMessage.id);
    
    console.log('发送消息 - ID:', userMessage.id, 'timestamp:', timestampKey, 'pendingMessages size:', this.pendingMessages.size);
    
    // 设置超时处理（30秒）
    const timeoutId = setTimeout(() => {
      this.handleMessageTimeout(userMessage.id, timestampKey);
    }, 30000);
    this.messageTimeouts.set(timestampKey, timeoutId);

    this.scrollToBottom();

    // 发送WebSocket消息
    try {
      const app = getApp();
      app.sendSocketMessage({
        type: 'ai_chat_request',
        message: inputText,
        timestamp: timestamp,  // 发送整数时间戳
        robot_id: app.globalData.robotId || 'robot_123'
      });

      console.log('AI聊天消息已发送:', inputText, 'timestamp:', timestamp);

    } catch (error) {
      console.error('发送消息失败:', error);
      this.handleSendError(userMessage.id, error, inputText);
    }
  },

  // 更新消息状态
  updateMessageStatus: function(messageId, status) {
    const messages = this.data.messages.map(msg => 
      msg.id === messageId ? {...msg, status} : msg
    );
    this.setData({ messages });
    console.log('消息状态已更新:', messageId, '->', status);
  },

  // 滚动到底部
  scrollToBottom: function() {
    const messages = this.data.messages;
    if (messages.length > 0) {
      const lastMessage = messages[messages.length - 1];
      this.setData({
        scrollIntoView: `msg-${lastMessage.id}`
      });
      
      // 备用滚动方案
      setTimeout(() => {
        this.setData({
          scrollTop: 9999999
        });
      }, 100);
    }
  },

  // 格式化时间 - 增强版
  formatTime: function(date) {
    const now = new Date();
    const today = new Date(now.getFullYear(), now.getMonth(), now.getDate());
    const messageDate = new Date(date.getFullYear(), date.getMonth(), date.getDate());
    
    // 判断是否为今天
    if (messageDate.getTime() === today.getTime()) {
      // 今天：只显示时分
      const hours = date.getHours().toString().padStart(2, '0');
      const minutes = date.getMinutes().toString().padStart(2, '0');
      return `${hours}:${minutes}`;
    } else {
      // 其他日期：显示月日时分
      const month = (date.getMonth() + 1).toString().padStart(2, '0');
      const day = date.getDate().toString().padStart(2, '0');
      const hours = date.getHours().toString().padStart(2, '0');
      const minutes = date.getMinutes().toString().padStart(2, '0');
      return `${month}-${day} ${hours}:${minutes}`;
    }
  },

  // 处理长消息显示
  getDisplayText: function(message) {
    if (!message.text) return '';
    
    const maxLength = 200;
    if (message.text.length <= maxLength) {
      return message.text;
    }
    
    return message.expanded ? message.text : message.text.substring(0, maxLength) + '...';
  },

  // 切换消息展开状态
  toggleMessage: function(e) {
    const messageId = e.currentTarget.dataset.id;
    const messages = this.data.messages.map(msg => {
      if (msg.id == messageId) {
        return { ...msg, expanded: !msg.expanded };
      }
      return msg;
    });
    
    this.setData({ messages });
  },

  // 检查是否需要展开按钮
  needExpandButton: function(text) {
    return text && text.length > 200;
  },

  // 添加消息到列表（带性能优化）
  addMessage: function(message) {
    const messages = [...this.data.messages, message];
    
    // 限制消息历史数量，避免内存过大（保留最近100条消息）
    const maxMessages = 100;
    if (messages.length > maxMessages) {
      messages.splice(0, messages.length - maxMessages);
    }
    
    this.setData({ messages });
    
    // 保存消息历史到本地存储（可选）
    this.saveMessageHistory(messages);
  },

  // 保存消息历史到本地存储
  saveMessageHistory: function(messages) {
    try {
      // 只保存最近20条消息到本地存储，避免存储过大
      const saveMessages = messages.slice(-20).map(msg => ({
        id: msg.id,
        type: msg.type,
        text: msg.text,
        time: msg.time,
        status: msg.status === 'sending' ? 'failed' : msg.status, // 发送中状态保存为失败
        expanded: false // 重置展开状态
      }));
      
      wx.setStorageSync('chat_messages', saveMessages);
    } catch (e) {
      console.warn('保存聊天记录失败:', e);
    }
  },

  // 加载消息历史
  loadMessageHistory: function() {
    try {
      const savedMessages = wx.getStorageSync('chat_messages');
      if (savedMessages && Array.isArray(savedMessages) && savedMessages.length > 0) {
        // 为历史消息重新生成ID，防止重复
        const processedMessages = savedMessages.map(msg => ({
          ...msg,
          id: generateUniqueMessageId(msg.type || 'unknown')
        }));
        
        // 直接使用历史消息，不保留默认欢迎消息
        this.setData({ messages: processedMessages });
        console.log('已加载聊天历史', savedMessages.length, '条消息');
        return true;
      }
    } catch (e) {
      console.warn('加载聊天记录失败:', e);
    }
    return false;
  },

  // 清空聊天记录
  clearMessages(){wx.showModal({title:'确认清空',content:'确定要清空所有聊天记录吗？',success:r=>r.confirm&&(this.setData({messages:[{id:generateUniqueMessageId('welcome'),type:'ai',text:'您好！我是AgriSage智能助手，很高兴为您服务。有什么问题可以随时问我！',time:this.formatTime(new Date()),status:'success',expanded:!1}]}),wx.removeStorageSync('chat_messages'),wx.showToast({title:'已清空聊天记录',icon:'success'}))})},

  // 计算消息统计信息
  getMessageStats: function() {
    const messages = this.data.messages;
    const userMessages = messages.filter(msg => msg.type === 'user').length;
    const aiMessages = messages.filter(msg => msg.type === 'ai').length;
    return { total: messages.length, user: userMessages, ai: aiMessages };
  },

  // 处理WebSocket消息
  onSocketMessage: function(data) {
    console.log('Chat页面收到WebSocket消息:', data.type, 'timestamp:', data.timestamp);
    
    if (data.type === 'ai_chat_response') {
      this.handleAIResponse(data);
    } else if (data.type === 'error' && data.context === 'ai_chat') {
      this.handleAIError(data);
    }
  },

  // 处理AI回复
  handleAIResponse: function(data) {
    console.log('收到AI回复:', data);
    
    // 确保timestamp存在
    if (!data.timestamp) {
      console.error('AI响应缺少timestamp');
      // 标记最后一个发送中的消息为成功
      this.markLastSendingMessageSuccess();
    } else {
      // 首先标记用户消息为发送成功
      this.markUserMessageSuccess(data.timestamp);
    }
    
    // 创建AI回复消息
    const aiMessage = {
      id: generateUniqueMessageId('ai'),
      type: 'ai',
      text: data.message || '抱歉，我没有理解您的问题。',
      time: this.formatTime(new Date()),
      status: 'success',
      expanded: false,
      serverTimestamp: data.timestamp // 记录服务器时间戳
    };

    // 添加AI回复到消息列表
    this.addMessage(aiMessage);
    
    // 更新发送状态
    this.setData({
      sending: false
    });

    // 滚动到底部显示新消息
    setTimeout(() => {
      this.scrollToBottom();
    }, 100);
    
    // 显示成功提示（可选）
    if (this.data.connected) {
      console.log('AI回复添加成功，消息长度:', aiMessage.text.length);
    }
  },

  // 处理AI错误
  handleAIError: function(data) {
    console.error('AI聊天错误:', data);
    
    // 标记用户消息为失败
    if (data.timestamp) {
      this.markUserMessageFailed(data.timestamp);
    } else {
      this.markLastSendingMessageFailed();
    }
    
    // 更新发送状态
    this.setData({
      sending: false
    });

    // 根据错误类型显示不同提示
    this.showErrorMessage(data);
  },

  // 显示错误消息
  showErrorMessage: function(errorData) {
    let title = 'AI服务暂时不可用';
    let duration = 3000;
    
    // 根据错误类型定制消息
    if (errorData.message) {
      if (errorData.message.includes('网络')) {
        title = '网络连接异常，请检查网络';
        duration = 4000;
      } else if (errorData.message.includes('超时')) {
        title = 'AI响应超时，请重试';
        duration = 3000;
      } else if (errorData.message.includes('服务')) {
        title = '服务暂时不可用，请稍后重试';
        duration = 4000;
      } else {
        title = errorData.message;
      }
    }

    wx.showToast({
      title: title,
      icon: 'none',
      duration: duration
    });
  },

  // 处理发送错误
  handleSendError: function(messageId, error, originalText) {
    console.error('发送消息失败:', error);
    
    this.updateMessageStatus(messageId, 'error');
    this.setData({
      sending: false
    });

    // 根据错误类型判断是否自动重试
    if (this.shouldAutoRetry(error)) {
      this.autoRetryMessage(messageId, originalText);
    } else {
      wx.showToast({
        title: '发送失败，点击消息可重试',
        icon: 'none',
        duration: 3000
      });
    }
  },

  // 判断是否应该自动重试
  shouldAutoRetry: function(error) {
    // 网络相关错误且重试次数未达上限
    return this.data.networkRetryCount < this.data.maxRetryCount && 
           (error.message && error.message.includes('network'));
  },

  // 自动重试发送消息
  autoRetryMessage: function(messageId, originalText) {
    const retryCount = this.data.networkRetryCount + 1;
    this.setData({ networkRetryCount: retryCount });
    
    wx.showToast({
      title: `自动重试中(${retryCount}/${this.data.maxRetryCount})...`,
      icon: 'loading',
      duration: 2000
    });

    setTimeout(() => {
      if (retryCount <= this.data.maxRetryCount) {
        this.setData({ inputText: originalText });
        this.sendMessage();
      }
    }, 2000);
  },

  // 重试连接
  retryConnection: function() {
    wx.showLoading({
      title: '重新连接中...',
      mask: true
    });

    // 尝试重新连接WebSocket
    const app = getApp();
    if (app.connectWebSocket) {
      app.connectWebSocket().then(() => {
        wx.hideLoading();
        wx.showToast({
          title: '连接成功',
          icon: 'success',
          duration: 2000
        });
        this.checkConnection();
      }).catch((error) => {
        wx.hideLoading();
        wx.showToast({
          title: '连接失败，请检查网络',
          icon: 'none',
          duration: 3000
        });
      });
    } else {
      // 如果没有重连方法，延迟后检查连接状态
      setTimeout(() => {
        wx.hideLoading();
        this.checkConnection();
        if (this.data.connected) {
          wx.showToast({
            title: '连接恢复',
            icon: 'success',
            duration: 2000
          });
        } else {
          wx.showToast({
            title: '连接失败，请重启应用',
            icon: 'none',
            duration: 3000
          });
        }
      }, 3000);
    }
  },

  // 标记用户消息为成功
  markUserMessageSuccess: function(serverTimestamp) {
    // 转换为字符串以确保一致性
    const timestampKey = serverTimestamp.toString();
    
    console.log('标记消息成功 - timestamp:', timestampKey, 'pendingMessages keys:', Array.from(this.pendingMessages.keys()));
    
    // 清除超时计时器
    this.clearMessageTimeout(timestampKey);
    
    // 根据timestamp精确匹配消息
    const messageId = this.pendingMessages.get(timestampKey);
    if (messageId) {
      console.log('找到匹配的消息ID:', messageId);
      this.updateMessageStatus(messageId, 'success');
      this.pendingMessages.delete(timestampKey);
    } else {
      console.warn('未找到匹配的待响应消息, timestamp:', timestampKey);
      // 备用方案：查找最后一个发送中的消息
      this.markLastSendingMessageSuccess();
    }
  },

  // 标记用户消息为失败
  markUserMessageFailed: function(serverTimestamp) {
    // 转换为字符串以确保一致性
    const timestampKey = serverTimestamp.toString();
    
    console.log('标记消息失败 - timestamp:', timestampKey);
    
    // 清除超时计时器
    this.clearMessageTimeout(timestampKey);
    
    // 根据timestamp精确匹配消息
    const messageId = this.pendingMessages.get(timestampKey);
    if (messageId) {
      this.updateMessageStatus(messageId, 'error');
      this.pendingMessages.delete(timestampKey);
    } else {
      // 备用方案：查找最后一个发送中的消息
      this.markLastSendingMessageFailed();
    }
  },

  // 标记最后一个发送中的消息为成功（备用方案）
  markLastSendingMessageSuccess: function() {
    const messages = this.data.messages;
    for (let i = messages.length - 1; i >= 0; i--) {
      if (messages[i].type === 'user' && messages[i].status === 'sending') {
        this.updateMessageStatus(messages[i].id, 'success');
        break;
      }
    }
  },

  // 标记最后一个发送中的消息为失败（备用方案）
  markLastSendingMessageFailed: function() {
    const messages = this.data.messages;
    for (let i = messages.length - 1; i >= 0; i--) {
      if (messages[i].type === 'user' && messages[i].status === 'sending') {
        this.updateMessageStatus(messages[i].id, 'error');
        break;
      }
    }
  },

  // 处理消息超时
  handleMessageTimeout: function(messageId, timestamp) {
    console.warn('消息发送超时:', messageId, timestamp);
    
    // 更新消息状态为失败
    this.updateMessageStatus(messageId, 'error');
    
    // 清理映射
    this.pendingMessages.delete(timestamp);
    this.messageTimeouts.delete(timestamp);
    
    // 更新发送状态
    this.setData({
      sending: false
    });
    
    // 显示超时提示
    wx.showToast({
      title: '发送超时，请检查网络连接',
      icon: 'none',
      duration: 3000
    });
  },

  // 清除消息超时计时器
  clearMessageTimeout: function(timestamp) {
    const timestampKey = timestamp.toString();
    const timeoutId = this.messageTimeouts.get(timestampKey);
    if (timeoutId) {
      clearTimeout(timeoutId);
      this.messageTimeouts.delete(timestampKey);
    }
  },

  // 清理所有待响应消息和计时器
  clearAllPendingMessages: function() {
    // 确保Map实例存在
    if (this.messageTimeouts) {
      // 清除所有超时计时器
      this.messageTimeouts.forEach(timeoutId => {
        clearTimeout(timeoutId);
      });
      
      // 清空映射
      this.messageTimeouts.clear();
    }
    
    if (this.pendingMessages) {
      this.pendingMessages.clear();
    }
    
    console.log('已清理所有待响应消息');
  },

  // 网络状态监控
  startNetworkMonitoring: function() {
    // 监听网络状态变化
    wx.onNetworkStatusChange((res) => {
      console.log('网络状态变化:', res);
      if (!res.isConnected) {
        this.setData({ connected: false });
        wx.showToast({
          title: '网络连接已断开',
          icon: 'none',
          duration: 3000
        });
      } else if (res.isConnected && !this.data.connected) {
        // 网络恢复，尝试重连WebSocket
        this.retryConnection();
      }
    });

    // 定期检查连接状态（每30秒）
    this.connectionCheckInterval = setInterval(() => {
      this.checkConnection();
    }, 30000);
  },

  // 停止网络监控
  stopNetworkMonitoring: function() {
    if (this.connectionCheckInterval) {
      clearInterval(this.connectionCheckInterval);
      this.connectionCheckInterval = null;
    }
    // 注意：wx.offNetworkStatusChange需要传入具体的回调函数，这里简化处理
  },

  // 输入内容实时验证
  onInputChange: function(e) {
    let inputText = e.detail.value;
    
    // 实时过滤特殊字符
    const filteredText = this.filterSpecialChars(inputText);
    if (filteredText !== inputText) {
      inputText = filteredText;
      // 更新输入框内容
      this.setData({ inputText: filteredText });
    }
    
    this.setData({
      inputText: inputText
    });

    // 实时长度提示
    if (inputText.length > 450) {
      const remaining = 500 - inputText.length;
      if (remaining <= 0) {
        wx.showToast({
          title: '已达到字数限制',
          icon: 'none',
          duration: 1000
        });
      } else {
        console.log(`还可输入${remaining}个字符`);
      }
    }
  },

  // 重发消息
  retryMessage: function(e) {
    const messageId = e.currentTarget.dataset.id;
    const message = this.data.messages.find(msg => msg.id == messageId);
    
    if (message && message.status === 'error') {
      console.log('重发消息:', message.text);
      
      // 重置重试计数
      this.setData({ networkRetryCount: 0 });
      
      // 重新设置输入框内容并发送
      this.setData({
        inputText: message.text
      });
      
      // 延迟一下确保输入框内容更新后再发送
      setTimeout(() => {
        this.sendMessage();
      }, 100);
    }
  }
});