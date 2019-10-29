---
title: Less.js User Guidance
published: true
---

# Less.js User Guidance

原文地址: https://juejin.im/post/5a2bc28f6fb9a044fe464b19

1. 安装及运行

   ```bash
   # 在命令行使用npm安装
   npm install -g less
   # 使用
   lessc styles.less styles.css
   ```

2. 变量

   1. 值变量:	以@开头定义变量

      ```less
      /* Less */
      @color: #999;
      @bgColor: skyblue;//不要添加引号
      @width: 50%;
      #wrap {
        color: @color;
        background: @bgColor;
        width: @width;
      }
      
      /* 生成后的 CSS */
      #wrap {
        color: #999;
        background: skyblue;
        width: 50%;
      }
      ```

   2. 选择器变量

      ```less
      /* Less */
      @mySelector: #wrap;
      @Wrap: wrap;
      @{mySelector}{ // 选择器名 必须使用大括号包裹
        color: #999;
        width: 50%;
      }
      .@{Wrap}{
        color:#ccc;
      }
      #@{Wrap}{
        color:#666;
      }
      
      /* 生成的 CSS */
      #wrap{
        color: #999;
        width: 50%;
      }
      .wrap{
        color:#ccc;
      }
      #wrap{
        color:#666;
      }
      ```

   3. 属性变量

      ```less
      /* Less */
      @borderStyle: border-style;
      @Soild:solid;
      #wrap{
        @{borderStyle}: @Soild;  // 属性名 必须使用大括号包裹
      }
      
      /* 生成的 CSS */
      #wrap{
        border-style:solid;
      }
      ```

   4. url 变量

      ```less
      /* Less */
      @images: "../img";//需要加引号
      body {
        background: url("@{images}/dog.png");//usl变量名 必须使用大括号包裹
      }
      
      /* 生成的 CSS */
      body {
        background: url("../img/dog.png");
      }
      ```

   5. 声明变量

   ```less
   /* Less */
   @background: {background:red;};
   #main{
       @background();
   }
   @Rules:{
       width: 200px;
       height: 200px;
       border: solid 1px red;
   };
   #con{
     @Rules();
   }
   
   /* 生成的 CSS */
   #main{
     background:red;
   }
   #con{
     width: 200px;
     height: 200px;
     border: solid 1px red;
   }
   ```

   6.  变量运算

      ```less
      /* Less */
      @width:300px;
      @color:#222;
      #wrap{
        width:@width-20;
        height:@width-20*5;
        margin:(@width-20)*5;
        color:@color*2;
        background-color:@color + #111;
      }
      
      /* 生成的 CSS */
      #wrap{
        width:280px;
        height:200px;
        margin:1400px;
        color:#444;
        background-color:#333;
      }
      ```

3. 嵌套

   1. & : 代表上层选择器的名字

      ```less
      /* Less */
      #header{
        &:after{
          content:"Less is more!";
        }
        .title{
          font-weight:bold;
        }
        &_content{ // 理解方式：直接把 & 替换成 #header
          margin:20px;
        }
      }
      /* 生成的 CSS */
      #header::after{
        content:"Less is more!";
      }
      #header .title{ // 嵌套了
        font-weight:bold;
      }
      #header_content{ // 没有嵌套！
          margin:20px;
      }
      ```

   2. 媒体查询

      ``` less
      /* Less */
      #main{
          //something...
      
          @media screen{
              @media (max-width:768px){
                width:100px;
              }
          }
          @media tv {
            width:2000px;
          }
      }
      /* 生成的 CSS */
      @media screen and (maxwidth:768px){
        #main{
            width:100px; 
        }
      }
      @media tv{
        #main{
          width:2000px;
        }
      }
      ```

4. 混合方法

   1. 无参数方法

      ```less
      /* Less */
      .card { // 等价于 .card()
          background: #f6f6f6;
          -webkit-box-shadow: 0 1px 2px rgba(151, 151, 151, .58);
          box-shadow: 0 1px 2px rgba(151, 151, 151, .58);
      }
      #wrap{
        .card;//等价于.card();
      }
      /* 生成的 CSS */
      #wrap{
        background: #f6f6f6;
        -webkit-box-shadow: 0 1px 2px rgba(151, 151, 151, .58);
        box-shadow: 0 1px 2px rgba(151, 151, 151, .58);
      }
      ```

   2. 默认参数方法

      ```less
      /* Less 可以使用默认参数, 如果没有传参数, 即视作使用默认参数 
       * @arguments 犹如 JS 中的 arguments 指代的是 全部参数。
       * 传的参数中 必须带着单位
       */
      /* Less */
      .border(@a:10px,@b:50px,@c:30px,@color:#000){
          border:solid 1px @color;
          box-shadow: @arguments; // 指代的是 全部参数
      }
      #main{
          .border(0px,5px,30px,red); // 必须带着单位
      }
      #wrap{
          .border(0px);
      }
      #content{
          .border; // 等价于 .border()
      }
      
      /* 生成的 CSS */
      #main{
          border:solid 1px red;
          box-shadow:0px,5px,30px,red;
      }
      #wrap{
          border:solid 1px #000;
          box-shadow: 0px 50px 30px #000;
      }
      #content{
          border:solid 1px #000;
          box-shadow: 10px 50px 30px #000;
      }
      ```

      其他请参考原文 https://juejin.im/post/5a2bc28f6fb9a044fe464b19

5. 继承

   1. extend 关键词

      ```less
      /* Less */
      .animation{
          transition: all .3s ease-out;
          .hide{
            transform:scale(0);
          }
      }
      #main{
          &:extend(.animation);
      }
      #con{
          &:extend(.animation .hide);
      }
      
      /* 生成后的 CSS */
      .animation,#main{
        transition: all .3s ease-out;
      }
      .animation .hide , #con{
          transform:scale(0);
      }
      ```

   2. all 全局搜索替换 

      ``` less
      /* Less */
      #main{
        width: 200px;
      }
      #main {
        &:after {
          content:"Less is good!";
        }
      }
      #wrap:extend(#main all) {}
      
      /* 生成的 CSS */
      #main,#wrap{
        width: 200px;
      }
      #main:after, #wrap:after {
          content: "Less is good!";
      }
      ```

6. 内置函数

   1. 类似其他语言的标准库 http://lesscss.cn/functions/

7. 其他

**注释**

- /* */ CSS原生注释，会被编译在 CSS 文件中。
- /   / Less提供的一种注释，不会被编译在 CSS 文件中。

**避免编译**

```
/* Less */
#main{
  width:~'calc(300px-30px)';
}

/* 生成后的 CSS */
#main{
  width:calc(300px-30px);
}
复制代码
```

结构： `~' 值 '`

**变量拼串**

在平时工作中，这种需求 太常见了。 在下面例子中， 实现了不同的 transtion-delay、animation、@keyframes

```less
.judge(@i) when(@i=1){
  @size:15px;
}
.judge(@i) when(@i>1){
  @size:16px;
}
.loopAnimation(@i) when (@i<16) {
  
  .circle:nth-child(@{i}){
      .judeg(@i);
      border-radius:@size @size 0 0;
      animation: ~"circle-@{i}" @duration infinite @ease;
      transition-delay:~"@{i}ms";
  }
  @keyframes ~"circle-@{i}" {
      // do something...
  }
  .loopAnimation(@i + 1);
}
```

结构： `~"字符@{变量}字符"`;

**使用 JS**

因为 Less 是由 JS 编写，所以 Less 有一得天独厚的特性：代码中使用 Javascript 。

```less
/* Less */
@content:`"aaa".toUpperCase()`;
#randomColor{
  @randomColor: ~"rgb(`Math.round(Math.random() * 256)`,`Math.round(Math.random() * 256)`,`Math.round(Math.random() * 256)`)";
}
#wrap{
  width: ~"`Math.round(Math.random() * 100)`px";
  &:after{
      content:@content;
  }
  height: ~"`window.innerHeight`px";
  alert:~"`alert(1)`";
  #randomColor();
  background-color: @randomColor;
}
/* 生成后的 CSS */

// 弹出 1
#wrap{
  width: 随机值（0~100）px;
  height: 743px;//由电脑而异
  background: 随机颜色;
}
#wrap::after{
  content:"AAA";
}
```

