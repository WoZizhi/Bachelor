import av
import time
import socket
import threading
import tkinter as tk
from PIL import Image,ImageTk                                                  

IP_vehicle = '172.20.11.104' 
Port_vehicle = 6000
IP_ground = '172.20.127.225'
Port_ground1 = 7000
Port_ground2 = 8000
Port_video = 8080
Buffer = 4096
N = 10
done = False

# 套接字绑定
udp_ground_client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

udp_ground_server1 = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
udp_ground_server1.bind((IP_ground,Port_ground1))
udp_ground_server2 = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
udp_ground_server2.bind((IP_ground,Port_video))
    
window = tk.Tk()
window.title('Ground')
window.geometry('850x400')
var = []
l_txt = [None]*N
l_recv = [None]*N

leftframe = tk.Frame(window)
leftframe.pack(side = 'left')

l_vedio = tk.Label(window,text='视频流',font=('楷体',14)).place(x=120,y=30)
l_message = tk.Label(window,text='位姿信息',font=('楷体',14)).place(x=540,y=30)
label_Image = tk.Label(leftframe,bg = 'white')
label_Image.pack()

l_txt[0] = tk.Label(window,text='电量',font=('楷体',14))
l_txt[1] = tk.Label(window,text='高度',font=('楷体',14))
l_txt[2] = tk.Label(window,text='横滚角',font=('楷体',14))
l_txt[3] = tk.Label(window,text='俯仰角',font=('楷体',14))
l_txt[4] = tk.Label(window,text='偏航角',font=('楷体',14))
l_txt[5] = tk.Label(window,text='地表速度',font=('楷体',14))
l_txt[6] = tk.Label(window,text='空中速度',font=('楷体',14))
l_txt[7] = tk.Label(window,text='X轴速度',font=('楷体',14))
l_txt[8] = tk.Label(window,text='Y轴速度',font=('楷体',14))
l_txt[9] = tk.Label(window,text='Z轴速度',font=('楷体',14))

for i in range(N):
    # 界面布局
    var.append(tk.StringVar())
    l_recv[i] = tk.Label(window,textvariable=var[i],bg='white',font=('Times New Romans',14),width=10,height=1)
    if i < 5:
        l_txt[i].place(x=380,y=90+50*i)
        l_recv[i].place(x=460,y=90+50*i)
    else:
        l_txt[i].place(x=620,y=90+50*(i-5))
        l_recv[i].place(x=710,y=90+50*(i-5))     
    
def ground_message_recv():
    while True:
        # 根据不同的首字母接收对应的信息
        data_recv_message = udp_ground_server1.recvfrom(Buffer)[0]
        var[0].set(data_recv_message.decode().partition('b')[2].partition('a')[0]+'%')
        var[1].set(data_recv_message.decode().partition('a')[2].partition('r')[0]+'m')
        var[2].set(data_recv_message.decode().partition('r')[2].partition('p')[0][:6]+'rad')
        var[3].set(data_recv_message.decode().partition('p')[2].partition('y')[0][:6]+'rad')
        var[4].set(data_recv_message.decode().partition('y')[2].partition('c')[0][:6]+'rad')
        var[5].set(data_recv_message.decode().partition('c')[2].partition('d')[0][:3]+'m/s')
        var[6].set(data_recv_message.decode().partition('d')[2].partition('j')[0][:3]+'m/s')
        var[7].set(data_recv_message.decode().partition('j')[2].partition('q')[0]+'m/s')
        var[8].set(data_recv_message.decode().partition('q')[2].partition('k')[0]+'m/s')
        var[9].set(data_recv_message.decode().partition('k')[2]+'m/s')

def change_second():
    # 一秒改变一次逻辑
    global done
    while True:
        if not done:
            done = True
        time.sleep(1)
    
# 显示视频流
def ground_video_recv(label_Image):
    global done
    print("decode_thread start!")
    codec = av.CodecContext.create("h264", "r")
    while True:
        data_recv_video = udp_ground_server2.recvfrom(Buffer)[0]
        packets = codec.parse(data_recv_video)
        for packet in packets:
            frames = codec.decode(packet)
            for frame in frames:
                imgtk = ImageTk.PhotoImage(frame.to_image())
                label_Image.config(image=imgtk)
                label_Image.image = imgtk
                if done:
                    frame.to_image().save(r'C:\Users\wzz\Desktop\photo\%03d.jpg' % (frame.index))
                    # 改变 done 的逻辑
                    done = False
        if not data_recv_video:
            break
    
def control():
    # 设定额外的小窗口用以进行键盘控制
    control_window = tk.Toplevel(window)
    control_window.geometry('500x200')
    control_window.title('Vehicle-Control')
    leftframe = tk.Frame(control_window)
    leftframe.pack(side='left')
    rightframe = tk.Frame(control_window)
    rightframe.pack(side='right')
    
    excuse_label0 = tk.Message(control_window,width=200,text='使用指南',font=('宋体',18))
    excuse_label0.place(x=80,y=20)
    excuse_label1 = tk.Message(control_window,width=200,text='q代表起飞、l代表着陆',font=('宋体',12))
    excuse_label1.place(x=50,y=80)
    excuse_label2 = tk.Message(control_window,width=200,text='Up代表上升、Down代表下降',font=('宋体',12))
    excuse_label2.place(x=35,y=110)
    excuse_label3 = tk.Message(control_window,width=200,text='ws代表前后、ad代表左右',font=('宋体',12))
    excuse_label3.place(x=45,y=140)
    
    # 标签记录历史命令，并设置滚动条
    cmd_history_label = tk.Message(rightframe,width=200,text='命令历史记录',font=('宋体',14))
    cmd_history_text = tk.Text(rightframe,bg='white',fg='black',font=('Arial',12),width=18,height=8)
    text_scroll = tk.Scrollbar(rightframe)
    cmd_history_text.config(yscrollcommand=text_scroll.set)
    text_scroll.config(command=cmd_history_text.yview)
    
    cmd_history_label.pack(side='top')
    text_scroll.pack(side='right',fill='y')
    cmd_history_text.pack(side='right')
    
    # 将命令发送至无人机端
    def ground_send(event):
        cmd_str = 'press  ' + event.keysym + '\n'
        cmd_history_text.insert('insert',cmd_str)
        cmd_history_text.see('end')
        print('press ' + event.keysym)
        udp_ground_client.sendto(event.keysym.encode(),(IP_vehicle,Port_vehicle))
        
    control_window.bind("<KeyPress>",ground_send)
    # threading.Thread(target=control).start()
    # control_window.mainloop()
    
control_button = tk.Button(window,text='进入飞行控制模式',font=("宋体",16),bg="yellow",command=control)
control_button.place(x=495, y=340)

# 设定三个线程，分别执行信息接收、信息发送与逻辑改变
threading.Thread(target=ground_message_recv).start()
threading.Thread(target=change_second).start()
threading.Thread(target=ground_video_recv,args=(label_Image,)).start()

window.mainloop()