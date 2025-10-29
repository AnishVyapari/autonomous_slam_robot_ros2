<!-- Glassmorphism Banner with Animated SVG -->
<div align="center">
  <svg width="800" height="200" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <linearGradient id="glassGrad" x1="0%" y1="0%" x2="100%" y2="100%">
        <stop offset="0%" style="stop-color:rgba(100,200,255,0.4);stop-opacity:1" />
        <stop offset="100%" style="stop-color:rgba(150,100,255,0.4);stop-opacity:1" />
      </linearGradient>
      <filter id="blur">
        <feGaussianBlur in="SourceGraphic" stdDeviation="2" />
      </filter>
    </defs>
    
    <!-- Glass Background -->
    <rect width="800" height="200" rx="20" fill="url(#glassGrad)" filter="url(#blur)" opacity="0.6"/>
    <rect width="800" height="200" rx="20" fill="none" stroke="rgba(255,255,255,0.5)" stroke-width="2"/>
    
    <!-- Robot Icon Animation -->
    <g id="robot">
      <circle cx="150" cy="100" r="40" fill="rgba(255,255,255,0.3)" stroke="#fff" stroke-width="2">
        <animate attributeName="cy" values="100;90;100" dur="2s" repeatCount="indefinite"/>
      </circle>
      <rect x="130" y="115" width="40" height="50" rx="5" fill="rgba(255,255,255,0.3)" stroke="#fff" stroke-width="2">
        <animate attributeName="y" values="115;105;115" dur="2s" repeatCount="indefinite"/>
      </rect>
      <circle cx="140" cy="90" r="5" fill="#4CAF50">
        <animate attributeName="opacity" values="1;0.3;1" dur="1.5s" repeatCount="indefinite"/>
      </circle>
      <circle cx="160" cy="90" r="5" fill="#4CAF50">
        <animate attributeName="opacity" values="1;0.3;1" dur="1.5s" repeatCount="indefinite"/>
      </circle>
    </g>
    
    <!-- SLAM Wave Animation -->
    <g id="slam-waves">
      <circle cx="650" cy="100" r="20" fill="none" stroke="rgba(76,175,80,0.6)" stroke-width="2">
        <animate attributeName="r" values="20;60;20" dur="3s" repeatCount="indefinite"/>
        <animate attributeName="opacity" values="1;0;1" dur="3s" repeatCount="indefinite"/>
      </circle>
      <circle cx="650" cy="100" r="30" fill="none" stroke="rgba(76,175,80,0.4)" stroke-width="2">
        <animate attributeName="r" values="30;70;30" dur="3s" begin="0.5s" repeatCount="indefinite"/>
        <animate attributeName="opacity" values="1;0;1" dur="3s" begin="0.5s" repeatCount="indefinite"/>
      </circle>
    </g>
    
    <!-- Title Text -->
    <text x="400" y="90" font-family="Arial, sans-serif" font-size="36" font-weight="bold" fill="#fff" text-anchor="middle">
      Autonomous SLAM Robot
    </text>
    <text x="400" y="120" font-family="Arial, sans-serif" font-size="18" fill="rgba(255,255,255,0.9)" text-anchor="middle">
      🤖 Mapping the Future, One Frame at a Time
    </text>
    <text x="400" y="145" font-family="Arial, sans-serif" font-size="14" fill="rgba(255,255,255,0.7)" text-anchor="middle">
      Powered by ROS2 | Real-time Navigation | Advanced SLAM
    </text>
  </svg>
</div>

---

## ✨ Glassmorphic Bouncy Loader Animation

```html
<!DOCTYPE html>
<html>
<head>
  <style>
    .glass-loader-container {
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    }
    
    .glass-loader {
      display: flex;
      gap: 12px;
    }
    
    .bouncy-dot {
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: rgba(255, 255, 255, 0.2);
      backdrop-filter: blur(10px);
      -webkit-backdrop-filter: blur(10px);
      border: 1px solid rgba(255, 255, 255, 0.3);
      box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
      animation: bounce 1.4s infinite ease-in-out;
    }
    
    .bouncy-dot:nth-child(1) {
      animation-delay: -0.32s;
      background: rgba(76, 175, 80, 0.3);
    }
    
    .bouncy-dot:nth-child(2) {
      animation-delay: -0.16s;
      background: rgba(33, 150, 243, 0.3);
    }
    
    .bouncy-dot:nth-child(3) {
      background: rgba(255, 152, 0, 0.3);
    }
    
    @keyframes bounce {
      0%, 80%, 100% {
        transform: translateY(0) scale(1);
        opacity: 0.7;
      }
      40% {
        transform: translateY(-30px) scale(1.2);
        opacity: 1;
      }
    }
    
    .loader-text {
      position: absolute;
      margin-top: 80px;
      color: rgba(255, 255, 255, 0.8);
      font-family: 'Arial', sans-serif;
      font-size: 16px;
      letter-spacing: 2px;
    }
  </style>
</head>
<body>
  <div class="glass-loader-container">
    <div style="position: relative;">
      <div class="glass-loader">
        <div class="bouncy-dot"></div>
        <div class="bouncy-dot"></div>
        <div class="bouncy-dot"></div>
      </div>
      <div class="loader-text">MAPPING...</div>
    </div>
  </div>
</body>
</html>
```

---

# 🤖 Autonomous SLAM Robot ROS2
- 
**TF errors**: Ensure all coordinate frames are properly published
### Useful Resources:
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Navigation2 Configuration Guide](https://navigation.ros.org/configuration/index.html)
- [SLAM Best Practices](https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md)
### Contributing:
Contributions are welcome! Please follow these steps:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request
---
## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
---
## 🤝 Acknowledgments
- ROS2 Development Team
- Nav2 Contributors
- SLAM Toolbox Maintainers
- Open Source Robotics Foundation
---
## 📞 Contact
**Project Maintainer**: Anish Vyapari
  **GitHub**: [@AnishVyapari](https://github.com/AnishVyapari)
⭐ Star this repository if you find it helpful!
