// 聊天页面 - AI集成优化版
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
        text: '您好！我是AgriSage智能助手，专门为农业采摘机器人提供服务。有什么问题可以随时问我！',
        time: '10:30',
        status: 'success',
        expanded: false
      }
    ],
    inputText: '',
    sending: false,
    aiThinking: false, // 新增：AI思考状态
    scrollTop: 0,
    connected: false,
    scrollIntoView: '',
    messageQueue: [], // 消息渲染队列
    lastSendTime: 0, // 上次发送消息时间（防止频繁发送）
    sendingCount: 0, // 连续发送计数
    networkRetryCount: 0, // 网络重试次数
    maxRetryCount: 3, // 最大重试次数
    aiResponseTimeout: 45000, // AI响应超时时间（45秒，比普通消息长）
    aiServiceStatus: 'unknown' // AI服务状态：available/unavailable/unknown
  },

  // 页面实例属性（不存储在data中）
  pendingMessages: null, // 待响应消息映射 {timestamp: messageId}
  messageTimeouts: null, // 消息超时计时器
  connectionCheckInterval: null, // 连接检查定时器
  aiThinkingTimer: null, // AI思考提示定时器

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
    // 清理AI思考定时器
    this.clearAIThinkingTimer();
  },

  onUnload: function() {
    console.log('Chat页面卸载');
    
    // 清理所有待响应消息和计时器
    this.clearAllPendingMessages();
    
    // 清理AI相关定时器
    this.clearAIThinkingTimer();
    
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
    
    // 检查AI服务状态
    this.checkAIServiceStatus();
    
    // 启动网络状态监控
    this.startNetworkMonitoring();
    
    // 初始化滚动位置
    setTimeout(() => {
      this.scrollToBottom();
    }, 300);
    
    console.log('Chat页面初始化完成', hasHistory ? '(含历史消息)' : '(仅默认消息)');
  },

  // 页面恢复显示
  resumePage: function() {
    // 重新注册页面（防止被其他页面覆盖）
    const app = getApp();
    app.globalData.chatPage = this;
    
    // 检查连接状态
    this.checkConnection();
    
    // 重新检查AI服务状态
    this.checkAIServiceStatus();
    
    // 滚动到底部显示最新消息
    this.scrollToBottom();
    
    // 如果有待发送的消息，恢复发送状态
    if (this.data.sending) {
      console.log('页面恢复时检测到发送中状态');
    }
  },

  // 检查AI服务状态
  checkAIServiceStatus: function() {
    // 发送AI服务状态检查请求
    const app = getApp();
    if (app.globalData.connected) {
      // 这里可以添加AI服务健康检查逻辑
      // 暂时假设AI服务可用
      this.setData({
        aiServiceStatus: 'available'
      });
    } else {
      this.setData({
        aiServiceStatus: 'unavailable'
      });
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
      this.setData({
        aiServiceStatus: 'unavailable'
      });
    }
  },

  // 输入验证（AI优化版）
  validateInput: function(inputText) {
    // 空消息检查
    if (!inputText) {
      return { valid: false, message: '请输入您的问题' };
    }

    // 正在发送检查
    if (this.data.sending) {
      return { valid: false, message: '消息发送中，请稍候' };
    }

    // AI思考中检查
    if (this.data.aiThinking) {
      return { valid: false, message: 'AI正在思考中，请稍候' };
    }

    // 长度限制检查（AI消息可以稍微长一些）
    if (inputText.length > 800) {
      return { valid: false, message: '问题内容过长，请控制在800字以内' };
    }

    // 连续空格或换行检查
    if (/^\s+$/.test(inputText)) {
      return { valid: false, message: '问题内容不能只包含空格' };
    }

    // 特殊字符过滤
    const filteredText = this.filterSpecialChars(inputText);
    if (filteredText !== inputText) {
      this.setData({ inputText: filteredText });
      return { valid: false, message: '问题包含特殊字符已自动过滤，请重新发送' };
    }

    // AI服务状态检查
    if (this.data.aiServiceStatus === 'unavailable') {
      return { valid: false, message: 'AI服务暂时不可用，请稍后重试' };
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
      this.setData({ 
        networkRetryCount: 0,
        aiServiceStatus: 'unavailable'
      });
      wx.showModal({
        title: 'AI服务连接异常',
        content: '网络连接已断开，AI服务暂时不可用，请检查网络后重试',
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

  // 检查发送频率（AI优化版）
  checkSendFrequency: function() {
    const now = Date.now();
    const timeDiff = now - this.data.lastSendTime;
    
    // AI消息间隔稍微长一些，避免频繁请求
    if (timeDiff < 3000) {
      wx.showToast({
        title: 'AI需要时间思考，请稍后再问',
        icon: 'none',
        duration: 2000
      });
      return false;
    }

    // 连续发送计数检查
    if (this.data.sendingCount >= 3) {
      wx.showToast({
        title: 'AI对话过于频繁，请稍作休息',
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

    // 15秒后重置发送计数
    setTimeout(() => {
      this.setData({
        sendingCount: Math.max(0, this.data.sendingCount - 1)
      });
    }, 15000);

    return true;
  },

  // 发送消息（AI优化版）
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
      timestamp: timestamp
    };

    // 添加消息到列表
    this.addMessage(userMessage);
    this.setData({
      inputText: '',
      sending: true,
      aiThinking: true // 设置AI思考状态
    });

    // 显示AI思考提示
    this.showAIThinkingIndicator();

    // 记录待响应消息
    const timestampKey = timestamp.toString();
    this.pendingMessages.set(timestampKey, userMessage.id);
    
    console.log('发送AI问题 - ID:', userMessage.id, 'timestamp:', timestampKey);
    
    // 设置AI专用超时处理（45秒）
    const timeoutId = setTimeout(() => {
      this.handleAITimeout(userMessage.id, timestampKey);
    }, this.data.aiResponseTimeout);
    this.messageTimeouts.set(timestampKey, timeoutId);

    this.scrollToBottom();

    // 发送WebSocket消息到AI服务
    try {
      const app = getApp();
      app.sendSocketMessage({
        type: 'ai_chat_request',
        message: inputText,
        timestamp: timestamp,
        robot_id: app.globalData.robotId || 'robot_123'
      });

      console.log('AI聊天问题已发送:', inputText.substring(0, 50), 'timestamp:', timestamp);

    } catch (error) {
      console.error('发送AI问题失败:', error);
      this.handleSendError(userMessage.id, error, inputText);
    }
  },

  // 显示AI思考指示器
  showAIThinkingIndicator: function() {
    // 添加AI思考消息
    const thinkingMessage = {
      id: generateUniqueMessageId('ai_thinking'),
      type: 'ai',
      text: 'AI正在思考中，请稍候...',
      time: this.formatTime(new Date()),
      status: 'thinking',
      expanded: false,
      isThinking: true // 标记为思考消息
    };

    this.addMessage(thinkingMessage);
    this.scrollToBottom();

    // 设置思考动画定时器
    this.aiThinkingTimer = setTimeout(() => {
      // 更新思考消息文本
      this.updateThinkingMessage('AI正在分析您的问题...');
      
      setTimeout(() => {
        this.updateThinkingMessage('AI正在生成回复...');
      }, 8000);
    }, 5000);
  },

  // 更新思考消息
  updateThinkingMessage: function(newText) {
    const messages = this.data.messages.map(msg => {
      if (msg.isThinking && msg.status === 'thinking') {
        return { ...msg, text: newText };
      }
      return msg;
    });
    this.setData({ messages });
  },

  // 移除思考消息
  removeThinkingMessage: function() {
    const messages = this.data.messages.filter(msg => !msg.isThinking);
    this.setData({ 
      messages,
      aiThinking: false
    });
    this.clearAIThinkingTimer();
  },

  // 清理AI思考定时器
  clearAIThinkingTimer: function() {
    if (this.aiThinkingTimer) {
      clearTimeout(this.aiThinkingTimer);
      this.aiThinkingTimer = null;
    }
  },

  // 处理AI超时
  handleAITimeout: function(messageId, timestamp) {
    console.warn('AI响应超时:', messageId, timestamp);
    
    // 移除思考消息
    this.removeThinkingMessage();
    
    // 更新消息状态为失败
    this.updateMessageStatus(messageId, 'error');
    
    // 清理映射
    this.pendingMessages.delete(timestamp);
    this.messageTimeouts.delete(timestamp);
    
    // 更新发送状态
    this.setData({
      sending: false,
      aiServiceStatus: 'unavailable'
    });
    
    // 显示AI专用超时提示
    wx.showModal({
      title: 'AI响应超时',
      content: 'AI服务响应时间过长，可能是网络问题或服务繁忙，是否重试？',
      confirmText: '重试',
      cancelText: '取消',
      success: (res) => {
        if (res.confirm) {
          // 找到失败的消息并重试
          const failedMessage = this.data.messages.find(msg => msg.id === messageId);
          if (failedMessage) {
            this.setData({ inputText: failedMessage.text });
            setTimeout(() => {
              this.sendMessage();
            }, 1000);
          }
        }
      }
    });
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
      // 只保存最近20条消息到本地存储，避免存储过大，过滤掉思考消息
      const saveMessages = messages
        .filter(msg => !msg.isThinking) // 不保存思考消息
        .slice(-20)
        .map(msg => ({
          id: msg.id,
          type: msg.type,
          text: msg.text,
          time: msg.time,
          status: msg.status === 'sending' ? 'failed' : msg.status,
          expanded: false
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
        
        // 检查是否有AI消息，如果有则更新AI服务状态为可用
        const hasAIMessages = processedMessages.some(msg => msg.type === 'ai' && !msg.isThinking);
        if (hasAIMessages) {
          this.setData({ aiServiceStatus: 'available' });
        }
        
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
  clearMessages: function() {
    wx.showModal({
      title: '确认清空',
      content: '确定要清空所有AI聊天记录吗？',
      success: (res) => {
        if (res.confirm) {
          // 清理思考状态
          this.removeThinkingMessage();
          
          this.setData({
            messages: [{
              id: generateUniqueMessageId('welcome'),
              type: 'ai',
              text: '您好！我是AgriSage智能助手，专门为农业采摘机器人提供服务。有什么问题可以随时问我！',
              time: this.formatTime(new Date()),
              status: 'success',
              expanded: false
            }],
            sending: false,
            aiThinking: false
          });
          
          wx.removeStorageSync('chat_messages');
          wx.showToast({
            title: '已清空聊天记录',
            icon: 'success'
          });
        }
      }
    });
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

  // 处理AI回复（优化版）
  handleAIResponse: function(data) {
    console.log('收到AI回复:', data);
    
    // 移除思考消息
    this.removeThinkingMessage();
    
    // 确保timestamp存在
    if (!data.timestamp) {
      console.error('AI响应缺少timestamp');
      this.markLastSendingMessageSuccess();
    } else {
      // 首先标记用户消息为发送成功
      this.markUserMessageSuccess(data.timestamp);
    }
    
    // 检查AI回复内容
    const aiResponseText = data.message || 'AI暂时无法回答您的问题，请稍后重试。';
    
    // 创建AI回复消息
    const aiMessage = {
      id: generateUniqueMessageId('ai'),
      type: 'ai',
      text: aiResponseText,
      time: this.formatTime(new Date()),
      status: 'success',
      expanded: false,
      serverTimestamp: data.timestamp,
      responseTime: Date.now() - (data.timestamp || Date.now()) // 计算响应时间
    };

    // 添加AI回复到消息列表
    this.addMessage(aiMessage);
    
    // 更新状态
    this.setData({
      sending: false,
      aiThinking: false,
      aiServiceStatus: 'available' // 成功响应说明AI服务可用
    });

    // 滚动到底部显示新消息
    setTimeout(() => {
      this.scrollToBottom();
    }, 100);
    
    // 记录统计信息
    console.log('AI回复添加成功，消息长度:', aiMessage.text.length, '响应时间:', aiMessage.responseTime, 'ms');
  },

  // 处理AI错误（优化版）
  handleAIError: function(data) {
    console.error('AI聊天错误:', data);
    
    // 移除思考消息
    this.removeThinkingMessage();
    
    // 标记用户消息为失败
    if (data.timestamp) {
      this.markUserMessageFailed(data.timestamp);
    } else {
      this.markLastSendingMessageFailed();
    }
    
    // 更新发送状态
    this.setData({
      sending: false,
      aiThinking: false,
      aiServiceStatus: 'unavailable'
    });

    // 根据错误类型显示不同提示
    this.showAIErrorMessage(data);
  },

  // 显示AI错误消息（优化版）
  showAIErrorMessage: function(errorData) {
    let title = 'AI服务暂时不可用';
    let content = '请稍后重试或联系技术支持';
    let duration = 3000;
    
    // 根据错误类型定制消息
    if (errorData.message) {
      if (errorData.message.includes('网络')) {
        title = 'AI服务网络异常';
        content = '网络连接不稳定，请检查网络设置';
        duration = 4000;
      } else if (errorData.message.includes('超时')) {
        title = 'AI响应超时';
        content = 'AI服务响应时间过长，请重试';
        duration = 3000;
      } else if (errorData.message.includes('服务')) {
        title = 'AI服务暂时不可用';
        content = '服务器忙碌或维护中，请稍后重试';
        duration = 4000;
      } else if (errorData.message.includes('API')) {
        title = 'AI服务配置错误';
        content = 'AI服务配置有误，请联系管理员';
        duration = 5000;
      } else {
        title = 'AI服务异常';
        content = errorData.message;
      }
    }

    // 使用模态框显示详细错误信息
    wx.showModal({
      title: title,
      content: content,
      showCancel: true,
      confirmText: '重试',
      cancelText: '取消',
      success: (res) => {
        if (res.confirm) {
          // 重试逻辑
          this.retryLastMessage();
        }
      }
    });
  },

  // 重试最后一条失败的消息
  retryLastMessage: function() {
    const messages = this.data.messages;
    for (let i = messages.length - 1; i >= 0; i--) {
      if (messages[i].type === 'user' && messages[i].status === 'error') {
        this.setData({ inputText: messages[i].text });
        setTimeout(() => {
          this.sendMessage();
        }, 500);
        break;
      }
    }
  },

  // 处理发送错误
  handleSendError: function(messageId, error, originalText) {
    console.error('发送AI问题失败:', error);
    
    // 移除思考消息
    this.removeThinkingMessage();
    
    this.updateMessageStatus(messageId, 'error');
    this.setData({
      sending: false,
      aiThinking: false,
      aiServiceStatus: 'unavailable'
    });

    // 显示发送失败提示
    wx.showToast({
      title: 'AI问题发送失败，点击消息可重试',
      icon: 'none',
      duration: 3000
    });
  },

  // 重试连接
  retryConnection: function() {
    wx.showLoading({
      title: '重新连接AI服务...',
      mask: true
    });

    // 尝试重新连接WebSocket
    const app = getApp();
    if (app.connectWebSocket) {
      app.connectWebSocket().then(() => {
        wx.hideLoading();
        wx.showToast({
          title: 'AI服务连接成功',
          icon: 'success',
          duration: 2000
        });
        this.checkConnection();
        this.checkAIServiceStatus();
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
            title: 'AI服务连接恢复',
            icon: 'success',
            duration: 2000
          });
          this.setData({ aiServiceStatus: 'available' });
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
    const timestampKey = serverTimestamp.toString();
    
    console.log('标记AI问题发送成功 - timestamp:', timestampKey);
    
    // 清除超时计时器
    this.clearMessageTimeout(timestampKey);
    
    const messageId = this.pendingMessages.get(timestampKey);
    if (messageId) {
      this.updateMessageStatus(messageId, 'success');
      this.pendingMessages.delete(timestampKey);
    } else {
      console.warn('未找到匹配的待响应AI消息, timestamp:', timestampKey);
      this.markLastSendingMessageSuccess();
    }
  },

  // 标记用户消息为失败
  markUserMessageFailed: function(serverTimestamp) {
    const timestampKey = serverTimestamp.toString();
    
    console.log('标记AI问题发送失败 - timestamp:', timestampKey);
    
    this.clearMessageTimeout(timestampKey);
    
    const messageId = this.pendingMessages.get(timestampKey);
    if (messageId) {
      this.updateMessageStatus(messageId, 'error');
      this.pendingMessages.delete(timestampKey);
    } else {
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
    if (this.messageTimeouts) {
      this.messageTimeouts.forEach(timeoutId => {
        clearTimeout(timeoutId);
      });
      this.messageTimeouts.clear();
    }
    
    if (this.pendingMessages) {
      this.pendingMessages.clear();
    }
    
    console.log('已清理所有待响应AI消息');
  },

  // 网络状态监控
  startNetworkMonitoring: function() {
    wx.onNetworkStatusChange((res) => {
      console.log('网络状态变化:', res);
      if (!res.isConnected) {
        this.setData({ 
          connected: false,
          aiServiceStatus: 'unavailable'
        });
        wx.showToast({
          title: 'AI服务网络连接已断开',
          icon: 'none',
          duration: 3000
        });
      } else if (res.isConnected && !this.data.connected) {
        this.retryConnection();
      }
    });

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
  },

  // 输入内容实时验证（AI优化）
  onInputChange: function(e) {
    let inputText = e.detail.value;
    
    // 实时过滤特殊字符
    const filteredText = this.filterSpecialChars(inputText);
    if (filteredText !== inputText) {
      inputText = filteredText;
      this.setData({ inputText: filteredText });
    }
    
    this.setData({
      inputText: inputText
    });

    // 实时长度提示（AI版本）
    if (inputText.length > 700) {
      const remaining = 800 - inputText.length;
      if (remaining <= 0) {
        wx.showToast({
          title: '问题内容已达到字数限制',
          icon: 'none',
          duration: 1000
        });
      } else {
        console.log(`AI问题还可输入${remaining}个字符`);
      }
    }
  },

  // 重发消息（AI优化版）
  retryMessage: function(e) {
    const messageId = e.currentTarget.dataset.id;
    const message = this.data.messages.find(msg => msg.id == messageId);
    
    if (message && message.status === 'error') {
      console.log('重发AI问题:', message.text);
      
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