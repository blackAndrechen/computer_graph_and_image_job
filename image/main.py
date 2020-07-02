import tkinter as tk
import tkinter.ttk as ttk
from PIL import Image, ImageTk
from tkinter import font,filedialog,messagebox
import cv2
import numpy as np
import random


def show_matrix(img):
    img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    tmp=Image.fromarray(img)
    tmp.show()



root=tk.Tk()
root.title("photo tools")


## 居中---------------------------------
# root.geometry("1230x625")
cur_width = 1230
cur_height = 625

scn_width, scn_height = root.maxsize()
tmpcnf = '+%d+%d' % ((scn_width-cur_width)/2, (scn_height-cur_height)/2)
root.geometry(tmpcnf)


## -------------------------------------


ft_menu = font.Font(family='newspaper', size=13, weight="normal")
ft_content=font.Font(family='newspaper', size=12, weight="normal")

s=ttk.Style()
# print(s.theme_names())
s.configure('.', font=ft_content)

# 菜单栏
menu_bar=tk.Menu(root)
root["menu"]=menu_bar

#标签 -------------------------------
notebook=ttk.Notebook(root,padding=2)

#菜单回调函数 -----------------------------------

def open_img():
    global img1,img2,img3,photo1,photo2,photo3

    _path=filedialog.askopenfilename()
    

    if _path.split(".")[-1] not in ["jpg","jpeg","png"]:
        messagebox.showwarning("警告", "请选择jpg,jpeg,png格式的图片")
        return
    
    if notebook.index("current")==0:
        _img = Image.open(_path)
        _img=_img.resize((600,400))
        
        img1=_img
        photo1=ImageTk.PhotoImage(_img)

        label01.configure(image=photo1)
    elif notebook.index("current")==1:
        _img = Image.open(_path)
        _img=_img.resize((400,400))
        
        img2=_img
        photo2=ImageTk.PhotoImage(_img)

        label11.configure(image=photo2)
    elif notebook.index("current")==2:
        _img = Image.open(_path)
        _img=_img.resize((600,400))
        
        img3=_img
        photo3=ImageTk.PhotoImage(_img)

        label31.configure(image=photo3)

def save_img():
    global result1_img,result2_img,result3_img
    _path=filedialog.asksaveasfilename()
    if _path.split(".")[-1]!="jpg":
        messagebox.showwarning("警告", "文件名需要为.jpg结尾")
        return
    if notebook.index("current")==0:
        img_tmp=cv2.cvtColor(result1_img,cv2.COLOR_BGR2RGB)
        cv2.imwrite(_path,img_tmp)
    elif notebook.index("current")==1:
        img_tmp=cv2.cvtColor(result2_img,cv2.COLOR_BGR2RGB)
        cv2.imwrite(_path,img_tmp)
    elif notebook.index("current")==2:
        img_tmp=cv2.cvtColor(result3_img,cv2.COLOR_BGR2RGB)
        cv2.imwrite(_path,img_tmp)

def exit_root():
    global root
    root.destroy()

def about_dis():
    about_root=tk.Toplevel(width=400,height=300)
    about_root.title("about")
    tmpcnf = '+%d+%d' % ((scn_width-400)/2, (scn_height-300)/2)
    about_root.geometry(tmpcnf)
    about_root.transient(root)
    # about_root.overrideredirect(True)
    
    def about_exit():
        about_root.destroy()

    label=tk.Label(about_root,text="说明",font=ft_content).grid(row=0,padx=20,pady=10,column=0,columnspan=2)
    label=tk.Label(about_root,text="本软件为计算机图形学结课作业",font=ft_content).grid(row=1,padx=20,columnspan=2,pady=10,column=0)

    label=tk.Label(about_root,text="author:",font=ft_content).grid(row=2,padx=20,pady=10,column=0)
    label=tk.Label(about_root,text="czp_chen",font=ft_content).grid(row=2,padx=20,pady=10,column=1)
    label=tk.Label(about_root,text="email:",font=ft_content).grid(row=3,padx=20,pady=10,column=0)
    label=tk.Label(about_root,text="980163660@qq.com",font=ft_content).grid(row=3,padx=20,pady=10,column=1)
    button=tk.Button(about_root,text="确定",font=ft_content,command=about_exit).grid(row=4,padx=20,pady=10,column=0,columnspan=2)

    about_root.mainloop()


def help_dis():
    help_root=tk.Toplevel(width=700,height=450)
    help_root.title("help")
    tmpcnf = '+%d+%d' % ((scn_width-700)/2, (scn_height-450)/2)
    help_root.geometry(tmpcnf)
    help_root.transient(root)
    # help_root.overrideredirect(True)
    
    def help_exit():
        help_root.destroy()

    tk.Label(help_root,text="").grid(row=0,column=0)
    tk.Label(help_root,text="人脸检测模块实现了Haar级联分类器用于人脸定位",font=ft_content).grid(row=1,column=0,sticky="S")
    tk.Message(help_root,width=680,text="参数说明：",font=ft_content).grid(row=2,column=0,sticky="W")
    tk.Message(help_root,width=680,text="阈值：构成检测目标的最邻矩形的最小个数，默认值为3，意为着有3个以上的检测标记存在时，才认为人脸存在。",font=ft_content).grid(sticky="W",row=3,column=0)
    tk.Message(help_root,width=680,text="最小半径：目标的最小尺寸，小于这个尺寸的目标将被忽略。",font=ft_content).grid(sticky="W",row=4,column=0)
    tk.Message(help_root,width=680,text="最大半径：目标的最大尺寸，最大半径需要大于最小半径。",font=ft_content).grid(sticky="W",row=5,column=0)
    tk.Message(help_root,width=680,text="扫描间隔：表示前后两次相继的扫描中，搜索窗口的缩放比例。",font=ft_content).grid(sticky="W",row=6,column=0)

    tk.Label(help_root,text="").grid(row=7,column=0)
    tk.Label(help_root,text="去噪模块实现了2种方式的加噪和3种方式的去噪",font=ft_content).grid(row=8,column=0,sticky="S")
    tk.Message(help_root,width=680,text="参数说明：",font=ft_content).grid(row=9,column=0,sticky="W")
    tk.Message(help_root,width=680,text="噪声密度：噪声占整个图片的比例",font=ft_content).grid(sticky="W",row=10,column=0)
    tk.Message(help_root,width=680,text="滤波器半径：高斯去噪和中值去噪的滤波器半径需要为奇数",font=ft_content).grid(sticky="W",row=11,column=0)
    
    tk.Label(help_root,text="").grid(row=12,column=0)
    tk.Label(help_root,text="去雾模块实现了暗通道去雾",font=ft_content).grid(row=13,column=0,sticky="S")
    tk.Message(help_root,width=680,text="参数说明：",font=ft_content).grid(row=14,column=0,sticky="W")
    tk.Message(help_root,width=680,text="Block大小：导向滤波器优化时的滤波器大小，当导向滤波器优化选项未勾选时该参数无效。",font=ft_content).grid(sticky="W",row=15,column=0)
    tk.Message(help_root,width=680,text="去雾程度：自然环境中为了使远处的风景保留一定的景深效果，我们需要保留一定的雾使得图片看得更加自然，该参数越大去雾越彻底。",font=ft_content).grid(sticky="W",row=16,column=0)
    tk.Message(help_root,width=680,text="最大大气光值：环境的大气光，暗通道去求得的大气光值不应大于u该值，否则以该值替换。",font=ft_content).grid(sticky="W",row=17,column=0)

    button=tk.Button(help_root,text="确定",font=ft_content,command=help_exit).grid(row=18,column=0)

    help_root.mainloop()

# -----------------------------------


file_menu=tk.Menu(menu_bar,tearoff=0)
menu_bar.add_cascade(label="文件",menu=file_menu,font=ft_menu)
file_menu.add_command(label='打开图片',font=ft_menu,command=open_img)
file_menu.add_command(label='保存图片',font=ft_menu,command=save_img)
file_menu.add_separator()# 添加分割线
file_menu.add_command(label='退出',font=ft_menu,command=exit_root)

about_menu = tk.Menu(menu_bar, tearoff=0)
menu_bar.add_cascade(label='关于', menu=about_menu,font=ft_menu)
about_menu.add_command(label='关于',font=ft_menu,command=about_dis)
about_menu.add_command(label='帮助',font=ft_menu,command=help_dis)






#标签一 -----------------------------
class face_dection:
    def __init__(self):
        self.faceCascade=cv2.CascadeClassifier("utils/haarcascade_frontalface_default.xml")

    def dection(self,img,minNeighbors=3,minSize=10,maxSize=500,scaleFactor=1.15):
        img_=img.copy()
        gray=cv2.cvtColor(img_,cv2.COLOR_BGR2GRAY)
    
        faces=self.faceCascade.detectMultiScale(gray,scaleFactor=scaleFactor,
            minNeighbors=minNeighbors,minSize=(minSize,minSize),maxSize=(maxSize,maxSize))

        for (x,y,w,h) in faces:
            cv2.rectangle(img_,(x,y),(x+w,y+w),(0,255,0),2)
            # cv2.circle(img_,(int((x+x+w)/2),int((y+y+h)/2)),int(w/2),(0,255,0),2)
        return img_

face_d=face_dection()

def detction(imgs,f1,min1,max1,speed1):
    global result1_pho,result1_img
    if max1.get()<min1.get():
        messagebox.showwarning("警告", "最大半径必须大于最小半径！")
        return
    
    result1_img=face_d.dection(np.array(imgs),f1.get(),min1.get(),max1.get(),speed1.get())
    result1_pho=ImageTk.PhotoImage(Image.fromarray(result1_img.astype("uint8")).convert("RGB"))
    label02.configure(image=result1_pho)



page1 = tk.Frame(notebook)

img1,photo1=None,None
f1,min1,max1,speed1=tk.IntVar(),tk.IntVar(),tk.IntVar(),tk.DoubleVar()
f1.set(3)
min1.set(5)
max1.set(200)
speed1.set(1.15)

path1="utils/face.jpeg"
img1 = Image.open(path1)
img1=img1.resize((600,400))
photo1 = ImageTk.PhotoImage(img1)

result1_img=face_d.dection(np.array(img1))
result1_pho=ImageTk.PhotoImage(Image.fromarray(result1_img.astype("uint8")).convert("RGB"))

tk.Label(page1,text="阈值:",font=ft_content).grid(row=0,column=0,sticky="E")
tk.Scale(page1,from_=2,to=8,variable=f1,resolution=1,orient='horizonta',length=200).grid(row=0,column=1,columnspan=2,sticky="W")

tk.Label(page1,text="最小半径:",font=ft_content).grid(row=1,column=0,sticky="E")
tk.Scale(page1,from_=0,to=50,variable=min1,resolution=1,orient='horizonta',length=200).grid(row=1,column=1,columnspan=2,sticky="W")

tk.Label(page1,text="最大半径:",font=ft_content).grid(row=2,column=0,sticky="E")
tk.Scale(page1,from_=20,to=200,variable=max1,resolution=1,orient='horizonta',length=200).grid(row=2,column=1,columnspan=2,sticky="W")

tk.Label(page1,text="扫描间隔:",font=ft_content).grid(row=3,column=0,sticky="E")
tk.Scale(page1,from_=1.05,to=2,variable=speed1,resolution=0.01,orient='horizonta',length=200).grid(row=3,column=1,columnspan=2,sticky="W")


tk.Button(page1,text="运行",font=ft_content,command=lambda:detction(img1,f1,min1,max1,speed1)).grid(row=0,column=3,rowspan=4,sticky="W")

label01=ttk.Label(page1,image=photo1)
label01.grid(row=4,column=0,columnspan=2,padx=5,pady=5)
label02=ttk.Label(page1,image=result1_pho)
label02.grid(row=4,column=2,columnspan=2,padx=5,pady=5)

ttk.Label(page1,text="原图",font=ft_content).grid(row=5,column=0,columnspan=2,padx=5,pady=5)
ttk.Label(page1,text="结果",font=ft_content).grid(row=5,column=2,columnspan=2,padx=5,pady=5)



#标签二 -------------------------------

class ARnoise:
    def __init__(self):
        pass

    def sp_noise(self,image,prob=0.05):
        '''
        添加椒盐噪声
        prob:噪声比例 
        '''
        output = np.zeros(image.shape,np.uint8)
        prob/=2
        thres = 1 - prob 
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                rdn = random.random()
                if rdn < prob:
                    output[i][j] = 0
                elif rdn > thres:
                    output[i][j] = 255
                else:
                    output[i][j] = image[i][j]
        return output

    def gasuss_noise(self,image, mean=0, var=0.01):
        ''' 
        添加高斯噪声
        mean : 均值 
        var : 方差
        '''
        image = np.array(image/255, dtype=float)
        noise = np.random.normal(mean, var ** 0.5, image.shape)
        out = image + noise
        out = np.clip(out, 0, 1.0)
        out = np.uint8(out*255)
        return out

    def remove_noise(self,img,method="medianblur",radius=3):
        # 均值滤波
        if method=="blur":
            img_ = cv2.blur(img, (radius,radius))

        elif method=="gaussianblur":
            img_ = cv2.GaussianBlur(img,(radius,radius),0)

        elif method=="medianblur":
            img_ = cv2.medianBlur(img, radius)

        return img_

noise=ARnoise()

def generate_noise(img,generate_type,generate_dens,generate_mean,generate_var):
    global mid_photo2,mid_img2
    if generate_type.get()=="椒盐噪声":
        mid_img2=noise.sp_noise(np.array(img),prob=generate_dens.get())
        mid_photo2=ImageTk.PhotoImage(Image.fromarray(mid_img2.astype("uint8")).convert("RGB"))
    else:
        mid_img2=noise.gasuss_noise(np.array(img),generate_mean.get(),generate_var.get())
        mid_photo2=ImageTk.PhotoImage(Image.fromarray(mid_img2.astype("uint8")).convert("RGB"))
    label12.configure(image=mid_photo2)


def remove_noise(img,remove_type,remove_rad):
    global result2_photo,result2_img
    if remove_type.get() in ["gaussianblur","medianblur"] and remove_rad.get()%2==0:
        messagebox.showwarning("警告", "高斯和中值去噪滤波器半径必须为奇数！")
        return

    
    result2_img=noise.remove_noise(img,remove_type.get(),remove_rad.get())

    result2_photo=ImageTk.PhotoImage(Image.fromarray(result2_img.astype("uint8")).convert("RGB"))
    label13.configure(image=result2_photo)

img2,mid_img2,result2_img=None,None,None
photo2,mid_photo2,result2_photo=None,None,None

generate_type,generate_dens,generate_mean,generate_var,remove_type,remove_rad=tk.StringVar(),tk.DoubleVar(),tk.DoubleVar(),tk.DoubleVar(),tk.StringVar(),tk.IntVar()
generate_dens.set(0.001)
generate_type.set("椒盐噪声")
remove_type.set("gaussianblur")
remove_rad.set(3)

path2="utils/23.jpg"
img2 = Image.open(path2)
img2=img2.resize((400,400))
photo2 = ImageTk.PhotoImage(img2)
mid_img2=noise.sp_noise(np.array(img2),prob=generate_dens.get())
mid_photo2=ImageTk.PhotoImage(Image.fromarray(mid_img2.astype("uint8")).convert("RGB"))
result2_img=noise.remove_noise(mid_img2)
result2_photo=ImageTk.PhotoImage(Image.fromarray(result2_img.astype("uint8")).convert("RGB"))

page2 = tk.Frame(notebook)

tk.Label(page2,text="噪声类型:",font=ft_content).grid(row=0,column=0,sticky="E",padx=0)
tk.Radiobutton(page2,font=ft_content,variable=generate_type,text="椒盐噪声",value="椒盐噪声").grid(row=0,column=1,sticky="W")
tk.Radiobutton(page2,font=ft_content,variable=generate_type,text="高斯噪声",value="高斯噪声").grid(row=0,column=1,sticky="E")

tk.Label(page2,text="椒盐噪声参数设置:",font=ft_content).grid(row=1,column=0,columnspan=2)
tk.Label(page2,text="噪声密度:",font=ft_content).grid(row=2,column=0,sticky="E",padx=0)
tk.Scale(page2,from_=0.001,to=1,variable=generate_dens,resolution=0.001,orient='horizonta',length=200).grid(row=2,column=1,sticky="W",padx=0)

tk.Label(page2,text="高斯噪声参数设置:",font=ft_content).grid(row=3,column=0,columnspan=2)
tk.Label(page2,text="噪声均值:",font=ft_content).grid(row=4,column=0,sticky="E",padx=0)
tk.Scale(page2,from_=0.001,to=0.1,variable=generate_mean,resolution=0.001,orient='horizonta',length=200).grid(row=4,column=1,sticky="W",padx=0)
tk.Label(page2,text="噪声方差:",font=ft_content).grid(row=5,column=0,sticky="E",padx=0)
tk.Scale(page2,from_=0.001,to=0.1,variable=generate_var,resolution=0.001,orient='horizonta',length=200).grid(row=5,column=1,sticky="W",padx=0)


tk.Button(page2,text="生成",font=ft_content,command=lambda:generate_noise(img2,generate_type,generate_dens,generate_mean,generate_var)).grid(row=0,column=2,rowspan=6,sticky="W")


tk.Label(page2,text="去噪方法:",font=ft_content).grid(row=0,column=3,sticky="E",padx=0)
tk.Radiobutton(page2,font=ft_content,variable=remove_type,text="高斯去噪",value="gaussianblur").grid(row=0,column=4,sticky="W")
tk.Radiobutton(page2,font=ft_content,variable=remove_type,text="均值去噪",value="blur").grid(row=0,column=4,sticky="E",padx=30)
tk.Radiobutton(page2,font=ft_content,variable=remove_type,text="中值去噪",value="medianblur").grid(row=0,column=5,sticky="W")


tk.Label(page2,text="滤波器半径:",font=ft_content).grid(row=1,column=3,rowspan=5,sticky="E",padx=0)
tk.Scale(page2,from_=3,to=9,variable=remove_rad,resolution=1,orient='horizonta',length=200).grid(row=1,rowspan=5,column=4,sticky="W",padx=0)

tk.Button(page2,text="运行",font=ft_content,command=lambda:remove_noise(mid_img2,remove_type,remove_rad)).grid(row=1,column=5,rowspan=5,sticky="W")



label11=ttk.Label(page2,image=photo2)
label11.grid(row=6,column=0,columnspan=2,padx=1)
label12=ttk.Label(page2,image=mid_photo2)
label12.grid(row=6,column=2,columnspan=2,padx=1)
label13=ttk.Label(page2,image=result2_photo)
label13.grid(row=6,column=4,columnspan=2,padx=1)

ttk.Label(page2,text="原图",font=ft_content).grid(row=7,column=0,columnspan=2,padx=1)
ttk.Label(page2,text="噪声图",font=ft_content).grid(row=7,column=2,columnspan=2,padx=1)
ttk.Label(page2,text="结果",font=ft_content).grid(row=7,column=4,columnspan=2,padx=1)



#标签三 -------------------------------

class dark_channel_prior_dehaze:
    def remove_haze(self,img,r=81,w=0.95,maxV1=0.80,guid=1):
        self.guid=guid
        m = self.deHaze(img/255.0,r=r,eps=0.001,w=w,maxV1=maxV1)*255
        return m.astype(np.uint8)

    def zmMinFilterGray(self,src, r=7):
        '''最小值滤波，r是滤波器半径'''
        '''if r <= 0:
            return src
        h, w = src.shape[:2]
        I = src
        res = np.minimum(I  , I[[0]+range(h-1)  , :])
        res = np.minimum(res, I[range(1,h)+[h-1], :])
        I = res
        res = np.minimum(I  , I[:, [0]+range(w-1)])
        res = np.minimum(res, I[:, range(1,w)+[w-1]])
        return zmMinFilterGray(res, r-1)'''
        return cv2.erode(src, np.ones((2*r+1, 2*r+1)))                      #使用opencv的erode函数更高效
    
    def guidedfilter(self,I, p, r, eps):
        '''引导滤波，直接参考网上的matlab代码'''
        height, width = I.shape
        m_I = cv2.boxFilter(I, -1, (r,r))
        m_p = cv2.boxFilter(p, -1, (r,r))
        m_Ip = cv2.boxFilter(I*p, -1, (r,r))
        cov_Ip = m_Ip-m_I*m_p
      
        m_II = cv2.boxFilter(I*I, -1, (r,r))
        var_I = m_II-m_I*m_I
      
        a = cov_Ip/(var_I+eps)
        b = m_p-a*m_I
      
        m_a = cv2.boxFilter(a, -1, (r,r))
        m_b = cv2.boxFilter(b, -1, (r,r))
        return m_a*I+m_b
      
    def getV1(self,m, r, eps, w, maxV1):  #输入rgb图像，值范围[0,1]
        '''计算大气遮罩图像V1和光照值A, V1 = 1-t/A'''
        V1 = np.min(m,2)  
        if self.guid==1:                                       #得到暗通道图像
            V1 = self.guidedfilter(V1, self.zmMinFilterGray(V1,7), r, eps)     #使用引导滤波优化
        else:
            V1=self.zmMinFilterGray(V1,7)
        bins = 2000
        #计算大气光照A
        ht = np.histogram(V1, bins)                              
        d = np.cumsum(ht[0])/float(V1.size)
        for lmax in range(bins-1, 0, -1): #取亮度前0.1%的柱状图横坐标，lmax右边的为所求
            if d[lmax]<=0.999:
                break
        A  = np.mean(m,2)[V1>=ht[1][lmax]].max()  #扁平化后取最大值
              
        V1 = np.minimum(V1*w, maxV1)                   #对值范围进行限制
          
        return V1,A
      
    def deHaze(self,m, r=81, eps=0.001, w=0.95, maxV1=0.80, bGamma=False):
        Y = np.zeros(m.shape)
        V1,A = self.getV1(m, r, eps, w, maxV1)               #得到遮罩图像和大气光照
        for k in range(3):
            Y[:,:,k] = (m[:,:,k]-V1)/(1-V1/A)           #颜色校正
        Y =  np.clip(Y, 0, 1)
        if bGamma:
            Y = Y**(np.log(0.5)/np.log(Y.mean()))       #gamma校正,默认不进行该操作
        return Y

dcpd=dark_channel_prior_dehaze()

def dehaze(img,haze_r,haze_w,haze_maxV1,haze_guid):
    global result3_photo,result3_img
    result3_img=dcpd.remove_haze(np.array(img),haze_r.get(),haze_w.get(),haze_maxV1.get()/255.0,haze_guid.get())

    result3_photo=ImageTk.PhotoImage(Image.fromarray(result3_img.astype("uint8")).convert("RGB"))

    label32.configure(image=result3_photo)



page3 = tk.Frame(notebook)

photo3,result3_photo,img3=None,None,None
haze_r,haze_w,haze_maxV1,haze_guid=tk.IntVar(),tk.DoubleVar(),tk.IntVar(),tk.IntVar()
haze_r.set(81)
haze_w.set(0.95)
haze_maxV1.set(204)
haze_guid.set(1)

path3="utils/house.jpg"
img3 = Image.open(path3)
img3=img3.resize((600,400))
photo3 = ImageTk.PhotoImage(img3)
result3_img=dcpd.remove_haze(np.array(img3))
result3_photo=ImageTk.PhotoImage(Image.fromarray(result3_img.astype("uint8")).convert("RGB"))


tk.Label(page3,text="Block大小:",font=ft_content).grid(row=0,column=0,sticky="E")
tk.Scale(page3,from_=10,to=100,variable=haze_r,resolution=1,orient='horizonta',length=200).grid(row=0,column=1,columnspan=2,sticky="W")

tk.Label(page3,text="去雾程度:",font=ft_content).grid(row=1,column=0,sticky="E")
tk.Scale(page3,from_=0,to=1,variable=haze_w,resolution=0.01,orient='horizonta',length=200).grid(row=1,column=1,columnspan=2,sticky="W")

tk.Label(page3,text="最大大气光值:",font=ft_content).grid(row=2,column=0,sticky="E")
tk.Scale(page3,from_=0,to=255,variable=haze_maxV1,orient='horizonta',length=200).grid(row=2,column=1,columnspan=2,sticky="W")

tk.Checkbutton(page3, text="使用导向滤波器优化",font=ft_content, variable=haze_guid).grid(row=3,column=1,rowspan=2,sticky="W",pady=5)

tk.Button(page3,text="运行",font=ft_content,command=lambda:dehaze(img3,haze_r,haze_w,haze_maxV1,haze_guid)).grid(row=0,column=3,rowspan=5,sticky="W")

label31=ttk.Label(page3,image=photo3)
label31.grid(row=5,column=0,columnspan=2,padx=5,pady=5)
label32=ttk.Label(page3,image=result3_photo)
label32.grid(row=5,column=2,columnspan=2,padx=5,pady=5)
ttk.Label(page3,text="原图",font=ft_content).grid(row=6,column=0,columnspan=2,padx=5,pady=5)
ttk.Label(page3,text="结果",font=ft_content).grid(row=6,column=2,columnspan=2,padx=5,pady=5)


# -------------------------------------
notebook.add(page1, text="人脸检测",padding=0)
notebook.add(page2, text="去噪")
notebook.add(page3, text="去雾")
notebook.pack(fill=tk.BOTH, expand="yes")

root.mainloop()
