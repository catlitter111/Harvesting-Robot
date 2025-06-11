// pages/statistics/statistics.js
Page({
    data: {
      // 日期相关
      currentDate: '',
      currentMonth: '',
      lastUpdateTime: '加载中...',
      
      // 概览统计
      todayHarvested: 0,
      todayArea: 0.0,
      todayHours: 0.0,
      totalHarvested: 0,
      totalArea: 0.0,
      totalHours: 0.0,
      harvestAccuracy: 0.0,
      
      // 设备健康状态
      batteryHealth: 0,
      cameraHealth: 92,  // 硬编码，实际应从设备获取
      armHealth: 78,     // 硬编码，实际应从设备获取
      
      // 导航和筛选
      currentTab: 'overview',
      filter: 'day',
      
      // 位置信息
      longitude: 108.2415,  // 默认经度
      latitude: 34.9385,    // 默认纬度
      locationName: '苹果园区3号地块 B-12 区域', // 默认位置名称
      formattedCoordinates: 'N 34°56\'12.3", E 108°14\'32.7"',
      movementStatus: '正在作业中 (0.5 km/h)',
      
      // 地图相关
      markers: [],
      polyline: [],
      polygons: [{
        points: [
          {latitude: 34.9395, longitude: 108.2403},
          {latitude: 34.9399, longitude: 108.2425},
          {latitude: 34.9380, longitude: 108.2430},
          {latitude: 34.9375, longitude: 108.2410}
        ],
        strokeWidth: 2,
        strokeColor: '#4CAF5080',
        fillColor: '#4CAF5020'
      }],
      historyPositions: [],  // 历史位置点
      
      // 路线历史
      routeHistory: [],
      currentRouteIndex: -1, // 当前位置在路线中的索引
      
      // 历史记录
      historyRecords: [],
      
      // 连接状态
      robotConnected: false,
      robotId: 'robot_123',
      
      // 数据是否已初始化
      dataInitialized: false,
      
      // 近7天数据表格
      weeklyData: [],
      weeklyTotal: {
        harvested: 0,
        area: 0.0,
        hours: 0.0,
        avgEfficiency: 0
      }
    },
  
    onLoad: function (options) {
      // 获取当前日期
      const today = new Date();
      const year = today.getFullYear();
      const month = today.getMonth() + 1;
      const day = today.getDate();
      
      const formattedDate = `${year}-${month < 10 ? '0' + month : month}-${day < 10 ? '0' + day : day}`;
      const formattedMonth = `${year}-${month < 10 ? '0' + month : month}`;
      
      this.setData({
        currentDate: formattedDate,
        currentMonth: formattedMonth,
        lastUpdateTime: this.formatTime(today)
      });
      
      // 注册到全局应用以接收消息
      const app = getApp();
      if (app.globalData) {
        app.globalData.statisticsPage = this;
      }
      
      // 初始化地图标记
      this.initMapMarkers();
      
      // 请求位置权限
      this.requestLocationPermission();
      
      // 请求最新统计数据
      this.requestStatisticsData();
      
      // 初始化近7天数据表格
      this.initWeeklyData();
      
      // 启动定时器刷新数据
      this.dataRefresher = setInterval(() => {
        this.requestStatisticsData();
      }, 30000);  // 每30秒刷新一次
    },
    
    // 请求位置权限
    requestLocationPermission: function() {
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
    
    // 初始化地图标记
    initMapMarkers: function() {
      const markers = [{
        id: 1,
        latitude: this.data.latitude,
        longitude: this.data.longitude,
        width: 40,
        height: 40,
        callout: {
          content: '采摘机器人',
          color: '#ffffff',
          fontSize: 12,
          borderRadius: 5,
          bgColor: '#4CAF50',
          padding: 5,
          display: 'ALWAYS'
        },
        iconPath: '/images/robot-marker.png'  // 需要准备一个机器人标记图标
      }];
      
      this.setData({ markers });
    },
    
    onShow: function() {
      // 页面显示时，再次注册以确保接收消息
      const app = getApp();
      if (app.globalData) {
        app.globalData.statisticsPage = this;
      }
      
      // 如果数据尚未初始化，请求最新数据
      if (!this.data.dataInitialized) {
        this.requestStatisticsData();
      }
      
      // 如果当前是位置标签页，更新地图显示
      if (this.data.currentTab === 'position') {
        this.updatePositionData();
      }
    },
    
    onHide: function() {
      // 页面隐藏时不做处理，保持WebSocket连接
    },
    
    onUnload: function() {
      // 页面卸载时，取消注册并清除定时器
      const app = getApp();
      if (app.globalData) {
        app.globalData.statisticsPage = null;
      }
      
      if (this.dataRefresher) {
        clearInterval(this.dataRefresher);
      }
    },
    
    // 格式化时间为 HH:MM 格式
    formatTime: function(date) {
      const hours = date.getHours().toString().padStart(2, '0');
      const minutes = date.getMinutes().toString().padStart(2, '0');
      return `${hours}:${minutes}`;
    },
    
    // 请求统计数据
    requestStatisticsData: function() {
      const app = getApp();
      if (!app.globalData || !app.globalData.connected) {
        console.warn('WebSocket未连接，无法请求统计数据');
        return;
      }
      
      // 发送数据请求
      app.sendSocketMessage({
        type: 'get_statistics',
        robot_id: this.data.robotId
      });
      
      // 同时请求位置数据
      app.sendSocketMessage({
        type: 'get_position',
        robot_id: this.data.robotId
      });
      
      // 请求历史轨迹数据
      app.sendSocketMessage({
        type: 'get_position_history',
        robot_id: this.data.robotId,
        date: this.data.currentDate
      });
      
      // 请求今日路线历史
      app.sendSocketMessage({
        type: 'get_route_history',
        robot_id: this.data.robotId,
        date: this.data.currentDate
      });
    },
    
    // 处理WebSocket消息
    onSocketMessage: function(data) {
      if (data.type === 'statistics_update') {
        this.updateStatisticsData(data.data);
      } else if (data.type === 'position_update') {
        this.updatePositionData(data.data);
      } else if (data.type === 'position_history') {
        // 接收历史位置数据
        this.setData({
          historyPositions: data.positions || []
        });
        
        // 如果当前是位置标签页，更新地图轨迹
        if (this.data.currentTab === 'position') {
          this.updatePositionTrack();
        }
      } else if (data.type === 'route_history') {
        // 处理路线历史数据
        if (data.routes && Array.isArray(data.routes)) {
          this.updateRouteHistory(data.routes);
        }
      }
    },
    
    // 更新统计数据
    updateStatisticsData: function(data) {
      if (!data) return;
      
      // 格式化数值
      const formatNumber = (num, digits = 1) => {
        if (num === undefined || num === null) return '0.0';
        return parseFloat(num).toFixed(digits);
      };
      
      // 获取当前时间作为更新时间
      const now = new Date();
      
      // 计算总面积和总时长（假设是每日数据的10倍和15倍，实际应从服务器获取）
      const totalArea = data.working_area ? parseFloat(data.working_area) * 10 : 0;
      const totalHours = data.working_hours ? parseFloat(data.working_hours) * 15 : 0;
      
      this.setData({
        todayHarvested: data.today_harvested || 0,
        todayArea: formatNumber(data.working_area || 0),
        todayHours: formatNumber(data.working_hours || 0),
        totalHarvested: data.total_harvested || 0,
        totalArea: formatNumber(totalArea, 1),
        totalHours: formatNumber(totalHours, 1),
        harvestAccuracy: formatNumber(data.harvest_accuracy || 95.0),
        batteryHealth: Math.round(data.battery_level || 0),
        lastUpdateTime: this.formatTime(now),
        robotConnected: true,
        dataInitialized: true
      });
      
      // 如果数据中包含位置信息，同时更新位置
      if (data.longitude !== undefined && data.latitude !== undefined) {
        this.setData({
          longitude: data.longitude,
          latitude: data.latitude
        });
        
        // 如果有位置名称，更新locationName
        if (data.location && data.location.name) {
          this.setData({
            locationName: data.location.name
          });
        }
        
        // 更新格式化的坐标
        const formattedCoordinates = this.formatCoordinates(data.latitude, data.longitude);
        this.setData({
          formattedCoordinates: formattedCoordinates
        });
        
        // 更新移动状态
        let movementStatus = '已停止';
        
        // 根据工作状态更新移动状态文本
        if (data.work_status) {
          if (data.work_status.mode === 'harvesting') {
            movementStatus = '正在采摘';
          } else if (data.work_status.mode === 'moving') {
            movementStatus = '正在移动中';
          } else if (data.work_status.mode === 'charging') {
            movementStatus = '正在充电中';
          } else {
            movementStatus = '待机中';
          }
          
          // 如果有速度信息，添加到移动状态中
          if (data.speed !== undefined) {
            movementStatus += ` (${data.speed.toFixed(1)} km/h)`;
          }
        }
        
        this.setData({
          movementStatus: movementStatus
        });
        
        // 如果当前是位置标签页，更新地图
        if (this.data.currentTab === 'position') {
          this.updatePositionData();
        }
      }
      
      // 处理路线历史数据
      if (data.route_history && Array.isArray(data.route_history)) {
        this.updateRouteHistory(data.route_history);
      }
      
      // 更新周数据表格
      this.updateWeeklyData();
      
      // 根据当前标签页加载相应数据
      if (this.data.currentTab === 'history') {
        this.loadHistoryData();
      }
    },
    
    // 更新位置数据
    updatePositionData: function(posData) {
      // 如果提供了位置数据参数，则使用参数数据
      if (posData) {
        // 更新经纬度
        if (posData.longitude !== undefined && posData.latitude !== undefined) {
          this.setData({
            longitude: posData.longitude,
            latitude: posData.latitude
          });
        }
        
        // 更新位置名称
        if (posData.location && posData.location.name) {
          this.setData({
            locationName: posData.location.name
          });
        } else if (posData.location_name) {
          this.setData({
            locationName: posData.location_name
          });
        }
        
        // 更新格式化坐标
        const formattedCoordinates = this.formatCoordinates(
          posData.latitude || this.data.latitude,
          posData.longitude || this.data.longitude
        );
        this.setData({
          formattedCoordinates: formattedCoordinates
        });
        
        // 更新移动状态
        let movementStatus = '已停止';
        
        // 根据工作状态更新移动状态文本
        if (posData.work_status) {
          if (posData.work_status.mode === 'harvesting') {
            movementStatus = '正在采摘';
          } else if (posData.work_status.mode === 'moving') {
            movementStatus = '正在移动中';
          } else if (posData.work_status.mode === 'charging') {
            movementStatus = '正在充电中';
          } else {
            movementStatus = '待机中';
          }
        }
        
        // 如果有速度信息，添加到移动状态中
        if (posData.speed !== undefined) {
          movementStatus += ` (${posData.speed.toFixed(1)} km/h)`;
        }
        
        this.setData({
          movementStatus: movementStatus
        });
        
        // 更新历史位置点
        if (posData.history_positions) {
          this.setData({
            historyPositions: posData.history_positions
          });
        }
        
        // 如果有路线历史数据，更新路线历史
        if (posData.route_history && Array.isArray(posData.route_history)) {
          this.updateRouteHistory(posData.route_history);
        }
      }
      
      // 更新标记
      const markers = this.data.markers;
      if (markers.length > 0) {
        markers[0].latitude = this.data.latitude;
        markers[0].longitude = this.data.longitude;
      }
      
      this.setData({
        markers
      });
      
      // 更新轨迹线
      this.updatePositionTrack();
      
      // 获取地图上下文并将视图中心设置为机器人位置
      const mapContext = wx.createMapContext('robotMap', this);
      mapContext.moveToLocation({
        longitude: this.data.longitude,
        latitude: this.data.latitude,
        fail: (err) => {
          console.error('移动地图失败', err);
        }
      });
    },
    
    // 更新轨迹线
    updatePositionTrack: function() {
      // 如果有历史轨迹点，构建路线
      if (this.data.historyPositions && this.data.historyPositions.length > 0) {
        const points = this.data.historyPositions.map(p => {
          return {
            longitude: p.longitude,
            latitude: p.latitude
          };
        });
        
        // 添加当前位置到路线尾部
        points.push({
          longitude: this.data.longitude,
          latitude: this.data.latitude
        });
        
        const polyline = [{
          points: points,
          color: '#4CAF50',
          width: 4,
          dottedLine: false,
          arrowLine: true
        }];
        
        this.setData({ polyline });
      } else {
        // 如果没有历史轨迹点，只显示当前位置
        this.setData({
          polyline: []
        });
      }
    },
    
    // 更新路线历史
    updateRouteHistory: function(routeHistory) {
      if (!routeHistory || !Array.isArray(routeHistory)) return;
      
      // 标准化数据格式，确保每个记录都有time和location字段
      const formattedHistory = routeHistory.map((item, index) => {
        return {
          time: item.time || '--:--',
          location: item.location || '未知位置',
          index: index // 添加索引以便标识当前位置
        };
      });
      
      // 确定当前位置在路线中的索引（通常是最后一个）
      const currentIndex = formattedHistory.length - 1;
      
      this.setData({
        routeHistory: formattedHistory,
        currentRouteIndex: currentIndex
      });
    },
    
    // 格式化坐标为人类可读格式
    formatCoordinates: function(latitude, longitude) {
      if (!latitude || !longitude) return 'N/A';
      
      const latDeg = Math.floor(Math.abs(latitude));
      const latMin = Math.floor((Math.abs(latitude) - latDeg) * 60);
      const latSec = ((Math.abs(latitude) - latDeg) * 60 - latMin) * 60;
      const latDir = latitude >= 0 ? 'N' : 'S';
      
      const lngDeg = Math.floor(Math.abs(longitude));
      const lngMin = Math.floor((Math.abs(longitude) - lngDeg) * 60);
      const lngSec = ((Math.abs(longitude) - lngDeg) * 60 - lngMin) * 60;
      const lngDir = longitude >= 0 ? 'E' : 'W';
      
      return `${latDir} ${latDeg}°${latMin}'${latSec.toFixed(1)}", ${lngDir} ${lngDeg}°${lngMin}'${lngSec.toFixed(1)}"`;
    },
    
    // 切换标签页
    switchTab: function(e) {
      const tab = e.currentTarget.dataset.tab;
      this.setData({
        currentTab: tab
      });
      
      // 根据标签页加载数据
      if (tab === 'overview') {
        // 概览数据已更新，无需额外处理
      } else if (tab === 'position') {
        this.updatePositionData();
      } else if (tab === 'history') {
        this.loadHistoryData();
      }
    },
    
    // 日期选择器变化
    dateChange: function(e) {
      this.setData({
        currentDate: e.detail.value
      });
      
      // 根据日期加载数据
      this.loadStatisticsData();
      
      // 如果当前是位置标签页，请求该日期的历史轨迹
      if (this.data.currentTab === 'position') {
        const app = getApp();
        app.sendSocketMessage({
          type: 'get_position_history',
          robot_id: this.data.robotId,
          date: e.detail.value
        });
        
        // 同时请求该日期的路线历史
        app.sendSocketMessage({
          type: 'get_route_history',
          robot_id: this.data.robotId,
          date: e.detail.value
        });
      }
    },
    
    // 月份选择器变化
    monthChange: function(e) {
      this.setData({
        currentMonth: e.detail.value
      });
      
      // 根据月份加载历史数据
      this.loadHistoryData();
    },
    
    // 设置过滤条件
    setFilter: function(e) {
      const filter = e.currentTarget.dataset.filter;
      this.setData({
        filter: filter
      });
      
      // 根据过滤条件重新加载历史数据
      this.loadHistoryData();
    },
    
    // 查看详情
    viewDetail: function(e) {
      const date = e.currentTarget.dataset.date;
      wx.navigateTo({
        url: `/pages/statistics/detail?date=${date}`
      });
    },
    
    // 加载统计数据（基于选择的日期）
    loadStatisticsData: function() {
      // 向服务器请求指定日期的数据
      // 现在只是简单模拟不同日期的数据变化
      console.log('加载日期为', this.data.currentDate, '的统计数据');
      
      // 生成随机变化的数据
      const baseData = this.data.dataInitialized ? {
        today_harvested: this.data.todayHarvested,
        working_area: parseFloat(this.data.todayArea),
        working_hours: parseFloat(this.data.todayHours)
      } : { today_harvested: 0, working_area: 0, working_hours: 0 };
      
      // 如果不是今天，则生成随机变化的历史数据
      const today = new Date().toISOString().split('T')[0];
      if (this.data.currentDate !== today) {
        const randomFactor = Math.random() * 0.5 + 0.75; // 0.75到1.25之间
        const mockData = {
          today_harvested: Math.round(baseData.today_harvested * randomFactor),
          working_area: baseData.working_area * randomFactor,
          working_hours: baseData.working_hours * randomFactor,
          harvest_accuracy: 95 + Math.random() * 5,
          battery_level: 85
        };
        this.updateStatisticsData(mockData);
      } else {
        // 如果是今天，请求实时数据
        this.requestStatisticsData();
      }
    },
    
    // 加载历史数据
    loadHistoryData: function() {
      console.log('加载历史数据，过滤方式:', this.data.filter);
      
      // 生成历史记录
      const today = new Date(this.data.currentDate);
      const records = [];
      
      // 根据当前月份生成历史数据
      const year = parseInt(this.data.currentMonth.split('-')[0]);
      const month = parseInt(this.data.currentMonth.split('-')[1]);
      
      // 基准数据
      const baseHarvested = this.data.dataInitialized ? this.data.todayHarvested : 250;
      const baseArea = this.data.dataInitialized ? parseFloat(this.data.todayArea) : 2.5;
      const baseHours = this.data.dataInitialized ? parseFloat(this.data.todayHours) : 8.0;
      
      // 生成本月的记录（最多5天）
      for (let i = 0; i < 5; i++) {
        // 避免生成未来的日期
        if (today.getDate() - i <= 0) continue;
        
        const recordDate = new Date(today.getFullYear(), today.getMonth(), today.getDate() - i);
        // 如果记录日期不在选择的月份内，则跳过
        if (recordDate.getMonth() + 1 !== month || recordDate.getFullYear() !== year) continue;
        
        const formattedDate = `${recordDate.getFullYear()}-${(recordDate.getMonth() + 1).toString().padStart(2, '0')}-${recordDate.getDate().toString().padStart(2, '0')}`;
        
        // 创建随机变化的历史数据
        const randomFactor = Math.random() * 0.3 + 0.85; // 0.85到1.15之间
        const dayFactor = 1 - i * 0.05; // 随着天数变化而递减
        
        records.push({
          date: formattedDate,
          day: recordDate.getDate(),
          month: recordDate.getMonth() + 1,
          title: '苹果园区作业',
          harvested: Math.round(baseHarvested * randomFactor * dayFactor),
          area: (baseArea * randomFactor * dayFactor).toFixed(1),
          hours: (baseHours * randomFactor * dayFactor).toFixed(1)
        });
      }
      
      this.setData({
        historyRecords: records
      });
    },
    
    // 地图控制方法
    zoomIn: function() {
      const mapContext = wx.createMapContext('robotMap', this);
      mapContext.getScale({
        success: (res) => {
          const newScale = Math.min(20, res.scale + 2);
          mapContext.includePoints({
            padding: [80, 80, 80, 80],
            points: [{
              latitude: this.data.latitude,
              longitude: this.data.longitude
            }],
            success: () => {
              setTimeout(() => {
                mapContext.moveToLocation({
                  longitude: this.data.longitude,
                  latitude: this.data.latitude
                });
              }, 100);
            }
          });
        }
      });
    },
    
    zoomOut: function() {
      const mapContext = wx.createMapContext('robotMap', this);
      mapContext.getScale({
        success: (res) => {
          const newScale = Math.max(5, res.scale - 2);
          mapContext.includePoints({
            padding: [120, 120, 120, 120],
            points: [{
              latitude: this.data.latitude, 
              longitude: this.data.longitude
            }]
          });
        }
      });
    },
    
    centerOnRobot: function() {
      const mapContext = wx.createMapContext('robotMap', this);
      mapContext.moveToLocation({
        longitude: this.data.longitude,
        latitude: this.data.latitude
      });
    },
    
    showWorkArea: function() {
      const mapContext = wx.createMapContext('robotMap', this);
      // 计算多边形的边界点
      let minLat = Number.MAX_VALUE, maxLat = Number.MIN_VALUE;
      let minLng = Number.MAX_VALUE, maxLng = Number.MIN_VALUE;
      
      this.data.polygons[0].points.forEach(point => {
        minLat = Math.min(minLat, point.latitude);
        maxLat = Math.max(maxLat, point.latitude);
        minLng = Math.min(minLng, point.longitude);
        maxLng = Math.max(maxLng, point.longitude);
      });
      
      // 将视图调整为包含整个作业区域
      mapContext.includePoints({
        padding: [60, 60, 60, 60],
        points: [
          {latitude: minLat, longitude: minLng},
          {latitude: maxLat, longitude: maxLng}
        ]
      });
    },
    
    // 初始化近7天数据表格
    initWeeklyData: function() {
      const weeklyData = [];
      const today = new Date();
      
      // 基准数据（可以从当前数据获取或使用默认值）
      const baseHarvested = 180;
      const baseArea = 2.1;
      const baseHours = 6.5;
      
      let totalHarvested = 0;
      let totalArea = 0;
      let totalHours = 0;
      let totalEfficiency = 0;
      
      // 生成近7天的数据
      for (let i = 6; i >= 0; i--) {
        const date = new Date(today);
        date.setDate(today.getDate() - i);
        
        // 根据日期生成不同的随机因子
        const dayFactor = 0.8 + Math.sin(i * 0.5) * 0.2; // 产生波动
        const randomFactor = 0.85 + Math.random() * 0.3;
        
        const harvested = Math.round(baseHarvested * dayFactor * randomFactor);
        const area = parseFloat((baseArea * dayFactor * randomFactor).toFixed(1));
        const hours = parseFloat((baseHours * dayFactor * randomFactor).toFixed(1));
        
        // 计算效率（采摘量/工作时间 的相对比例）
        const efficiency = Math.min(100, Math.round((harvested / (hours * 20)) * 100));
        
        // 格式化日期显示
        const dateDisplay = i === 0 ? '今日' : 
                           i === 1 ? '昨日' : 
                           `${date.getMonth() + 1}/${date.getDate()}`;
        
        const dayData = {
          date: date.toISOString().split('T')[0],
          dateDisplay: dateDisplay,
          harvested: harvested,
          area: area,
          hours: hours,
          efficiency: efficiency
        };
        
        weeklyData.push(dayData);
        
        // 累加统计
        totalHarvested += harvested;
        totalArea += area;
        totalHours += hours;
        totalEfficiency += efficiency;
      }
      
      // 计算周总计
      const weeklyTotal = {
        harvested: totalHarvested,
        area: totalArea.toFixed(1),
        hours: totalHours.toFixed(1),
        avgEfficiency: Math.round(totalEfficiency / 7)
      };
      
      this.setData({
        weeklyData: weeklyData,
        weeklyTotal: weeklyTotal
      });
    },
    
    // 更新周数据表格（当有新的当日数据时调用）
    updateWeeklyData: function() {
      const weeklyData = [...this.data.weeklyData];
      const today = new Date().toISOString().split('T')[0];
      
      // 查找今日数据
      const todayIndex = weeklyData.findIndex(item => item.date === today);
      
      if (todayIndex !== -1) {
        // 更新今日数据
        const todayHours = parseFloat(this.data.todayHours) || 1;
        const todayHarvested = this.data.todayHarvested || 0;
        const efficiency = Math.min(100, Math.round((todayHarvested / (todayHours * 20)) * 100));
        
        weeklyData[todayIndex] = {
          ...weeklyData[todayIndex],
          harvested: todayHarvested,
          area: parseFloat(this.data.todayArea) || 0,
          hours: todayHours,
          efficiency: efficiency
        };
        
        // 重新计算周总计
        let totalHarvested = 0;
        let totalArea = 0;
        let totalHours = 0;
        let totalEfficiency = 0;
        
        weeklyData.forEach(day => {
          totalHarvested += day.harvested;
          totalArea += parseFloat(day.area);
          totalHours += parseFloat(day.hours);
          totalEfficiency += day.efficiency;
        });
        
        const weeklyTotal = {
          harvested: totalHarvested,
          area: totalArea.toFixed(1),
          hours: totalHours.toFixed(1),
          avgEfficiency: Math.round(totalEfficiency / 7)
        };
        
        this.setData({
          weeklyData: weeklyData,
          weeklyTotal: weeklyTotal
        });
      }
    }
  });