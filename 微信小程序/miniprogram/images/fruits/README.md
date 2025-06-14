# 水果图片库说明

## 📁 目录结构
```
images/fruits/
├── apple_gala.jpg          # 嘎啦苹果
├── apple_fuji.jpg          # 红富士苹果  
├── apple_green.jpg         # 青苹果
├── orange.jpg              # 橙子
├── lemon.jpg               # 柠檬
├── banana.jpg              # 香蕉
├── strawberry.jpg          # 草莓
├── grape.jpg               # 葡萄
├── peach.jpg               # 桃子
├── pear.jpg                # 梨
├── cherry.jpg              # 樱桃
├── kiwi.jpg                # 猕猴桃
├── mango.jpg               # 芒果
├── pineapple.jpg           # 菠萝
├── watermelon.jpg          # 西瓜
├── sweet_orange.jpg        # 甜橙
└── default_fruit.jpg       # 默认水果图片
```

## 🖼️ 图片要求
- **格式**: JPG/PNG
- **尺寸**: 建议 300x300 像素以上
- **大小**: 每张图片不超过 500KB
- **质量**: 清晰、光线良好的水果照片

## 📝 添加新水果图片步骤

### 1. 准备图片文件
将水果图片放入此目录，命名格式：`水果名称.jpg`

### 2. 更新配置
在 `detection.js` 的 `fruitImageLibrary` 中添加新条目：
```javascript
fruitImageLibrary: {
  // 现有配置...
  '新水果名称': '/images/fruits/新水果图片.jpg'
}
```

### 3. 图片优化建议
- 使用图片压缩工具减小文件大小
- 确保图片主体居中、背景简洁
- 避免过度曝光或阴影过重

## 🔄 自动匹配规则
系统会按以下顺序匹配图片：
1. **精确匹配**: 水果名称完全一致
2. **模糊匹配**: 包含关键词的匹配
3. **默认图片**: 未找到匹配时使用

## 📱 使用示例
```javascript
// 在代码中调用
const imagePath = this.getLocalFruitImage('嘎啦苹果');
// 返回: '/images/fruits/apple_gala.jpg'
```

## ⚠️ 注意事项
- 图片文件名不要包含中文字符
- 确保图片文件存在，否则会显示默认图片
- 定期清理不使用的图片文件
- 图片版权请确保合规使用

## 🛠️ 维护建议
- 定期检查图片加载情况
- 根据识别结果添加新的水果类型
- 优化图片质量和大小
- 备份重要的图片文件 