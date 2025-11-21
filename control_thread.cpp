#include "control_thread.h"
#include <cmath>
#include "mainwindow.h"
#include "tracking_params.h" // 引入跟踪参考值
#include "control_alg_params.h" // 控制算法全局参数

extern quint8 flag_save;
extern float distance[4];
extern quint8 rsv_data[18];//img_data
extern quint8 cmd[42];//控制指令
extern QVector<QVector<double>> Rov_location;
extern QVector<QVector<double>> P_location;
extern float mpc_u; // MPC回传的期望速度
extern float mpc_r;
extern float mpc_tau_u; // MATLAB回传的期望力
extern float mpc_tau_r;
float mpc_u_e;//速度偏差量
float mpc_u_e_last;
float mpc_r_e;
float mpc_r_e_last;
float psi_e;
float psi_e_last;
// matlab接收同步标记
float matlab_rsv_flag = 0;
// 控制参数
extern float Ts;// 控制周期
int flag = 0;//控制状态标志位
int flag2 = 0;
quint8 flag_control = 0;
quint8 delta_v = 0;//warm up 控制量
quint8 avoid_v = 10;//避障速度

//APF 变量
//存储位置向量
QVector<QVector<float>> path;//APF 规划的路径
QVector<QVector<float>> path_simple;// 间距采样后，实际跟踪的路径
QVector<QVector<float>> path_final;// 每次重规划拼接的路径，后处理用
QVector<QVector<float>> target;
QVector<QVector<float>> obstacle;
QVector<QVector<int>> map;// 栅格地图，0表示空闲，1表示占用
// 目标点初始坐标
float goalX = -10;
float goalY = 10;
// 障碍物坐标
int obs_count = 3;
float obstacleX,obstacleY,obstacleX_2,obstacleY_2,obstacleX_3, obstacleY_3;
// 斥力半径
float rou_0 = 3;
// APF路径生成步长
const float path_len = 0.1;
// APF最大迭代次数
const int iter_len = 500;
// APF路径生成点数
int real_len = 0;
int simple_len = 0;
int final_len = 0;
// APF重规划标记位
int replan = 0;
// APF路径点位置
float pathX = 0.0;
float pathY = 0.0;
float pathX_last = 0.0;
float pathY_last = 0.0;
float pathX_last_last = 0.0;
float pathY_last_last = 0.0;
// APF合力的单位向量
float uint_F_X = 0.0;
float uint_F_Y = 0.0;
float uint_F_X_last = 0.0;
float uint_F_Y_last = 0.0;
// LOS路径跟踪全局变量
float p_x, p_y, p_next_x, p_next_y, x_los, y_los, s, afa_k, delta_e, Lpp = 0.6, R = 1.2, dist_D = 1.6;// Lpp是船长，R是LOS包围圆的半径，dist_D是路径点间距
float x, y, a, b, c, d, e, f, g, dx, dy;// LOS参数
// 到达判定
double k = 0, dist_goal, dist_IOU = 0.5;// 默认终点半径1米内无障碍物，便于刹车


int i = 1;
int num_i = 1;//当前巡点序号
int num_end = 7;//巡点数量上限
extern float psi_def = 1.5;//阈值
extern float Rov_x;//机器人坐标, 同 x,y
extern float Rov_y;
extern float v_x;
extern float v_y;
extern float Rov_u;//机器人速度u
extern float Rov_v;//机器人速度v
extern float Rov_r;//机器人速度r, 同 Rov_psi_diff
extern float Rov_psi;
extern float Rov_psi_last;
extern float Rov_psi_diff; // r
extern float Rov_psi_diff_last;
extern float Rov_psi_diff_2; // r'
float P_x = 0;//目标点坐标 LOS点或预设点
float P_y = 0;
float delta_x = 0;//坐标偏差
float delta_y = 0;
float delta_d = 0;//De
float delta_d_last = 0;
float delta_d_diff = 0;//De'
float P_psi = 0;
float psi_d = 0;//目标偏航角
float psi_d_last = 0;
float psi_d_diff_last = 0;
float psi_d_diff = 0;
float psi_d_diff_2 = 0;
float delta_psi = 0;//偏航角偏差
float delta_psi_last = 0;
float delta_psi_diff = 0;
//parameters
float d_t = 0.05; // 时间步长，单位s
float cruise_speed = 25;//巡航速度
float dis_def_navigate = 0.4;//m 到达终点的判定范围
float dis_def_collecte = 350;//mm 到达垃圾点的判定范围
float tau_n = 0;//转向力矩，控制量
float tau_n_last = 0;
float F = 0;//推进器推力，N
float l = 0.21;//推进器力臂, m
float TAU_N = 0;//转向力矩，N/m
//PID
float kp = 3;
float kd = 10;
float ki = 0;
float t = 0;
float sum = 0;// PID积分
//SMC
int motor_value = 0;//电机的控制量
float speed_u = 0;//前进的电机数值
float speed_r = 0;//转向的电机数值
float m_u = 32.6;
float m_v = 32.6;
float m_r =4;//2.4
float d_r = 0.1;
float S = 0;
float C0 = 40;
float lambda_0 =0.6;
float sigma = 0.15;
//轨迹跟踪
int timestep;
float w = 0.10;//圆轨迹角速度
float RX,RY;


//保存结果 psi, tau_letf, tau_right
QFile file1("D:\\USV_data\\control_data.txt");
QTextStream out(&file1);
QFile file2("D:\\USV_data\\path.txt");
QTextStream out2(&file2);
QFile file3("D:\\USV_data\\path_simple.txt");
QTextStream out3(&file3);

ControlThread::ControlThread(QObject *parent)
    : QObject{parent}
{
    timer_control = new QTimer(this);//control定时器初始化
    my_eso = new ESO;

    if(!file1.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << ("打开文件1失败");
    }
    if(!file2.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << ("打开文件2失败");
    }
    if(!file3.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << ("打开文件3失败");
    }
    //路径向量初始化为500行，2列
    path.resize(iter_len + 1);
    path_simple.resize(iter_len + 1);
    path_final.resize(iter_len + 1);
    for (int j = 0; j < path.size(); j++)
    {
        path[j].resize(2);//初始化向量组
        path_simple[j].resize(2);
        path_final[j].resize(2);
    }
    obstacle.resize(obs_count);// 最多可存储3个障碍物位置，后续可增加
    for (int j = 0; j < obstacle.size(); j++)
    {
        obstacle[j].resize(2);
    }
    obstacle[0][0] = -1;//虚拟障碍物坐标设定
    obstacle[0][1] = 7;
    obstacle[1][0] = -4;
    obstacle[1][1] = 3;
    obstacle[2][0] = -5;
    obstacle[2][1] = 8;

//    // 栅格地图初始化，d=0.5m，160*0.5=80， D=80m
//    map.resize(240 + 1);
//    for (int j = 0; j < map.size(); j++)
//    {
//        map[j].resize(240);// 160*160
//    }
    // 生成全局路径点，打印输出
    pathX = -0.01;//设置规划起点
    pathY = -0.05;
    APF_Generate();

    // 初始化pk、pk+1
    if (simple_len < 2)
        qDebug() << "Too few points!" << endl;
    p_x = path_simple[0][0];
    p_y = path_simple[0][1];
    p_next_x = path_simple[1][0];
    p_next_y = path_simple[1][1];

    for(int i=0;i<real_len;i++)
        out2 << path[i][0] << " " << path[i][1] << endl;
    for(int i=0;i<simple_len;i++)
        out3 << path_simple[i][0] << " " << path_simple[i][1] << endl;


    connect(timer_control,&QTimer::timeout, [=]()
    {
        timer_control->stop();

        x = Rov_x;// 读取机器人当前坐标，los法求解才用到
        y = Rov_y;

        // 更新正弦参考（集中函数），确保控制线程也驱动参考演化
        update_tracking_refs(Ts);
        // 调试输出（每秒一次）
        static double dbg_accum = 0.0; dbg_accum += Ts;
        if(dbg_accum >= 1.0) { dbg_accum = 0.0; qDebug() << "[REF] r_amp=" << ref_r_amp << " r_ref=" << ref_r; }

        // 期望速度差值
        //mpc_u = 0;//调试定速跟踪，用matlab要删掉
//        mpc_r = 0 + 0.2*sin(2*M_PI*t / 40);
//        t = t + Ts;

        //更新ESO,tau应该是由螺旋桨推力产生的力矩，需解算
        my_eso->eso_calculate(Rov_psi,m_r,motor_value,Rov_r);

        //QMutexLocker locker(&myMutex_2);//加锁,Rov_location[][]
        // 巡点控制才用到
//        x_los = P_location[num_i][0];// x对应0, y对应1
//        y_los = P_location[num_i][1];
//        delta_d = sqrt(pow(x_los-x,2) + pow(y_los-y,2));//到预设目标点的距离
//        qDebug()<<x_los<<" "<<y_los<<delta_d;
        //qDebug()<<delta_d;

        //judge
//        if(distance[0] <= 0.01||distance[1] <= 0.01||distance[2] <= 0.01||distance[3] <= 0.01)  //When d > 0.5m, how to add random force?
//        {
//            //qDebug()<<"nice";
//            if(flag == 0)
//                flag_control = 9;//stop 初始离障碍物近，则保持静止
//            else
//                flag_control = 8;//avoid 运动过程离障碍物近，则开启避障
//        }
//        else
//        {
            if(flag == 0)
                flag_control = 0;//warm up
            else if(flag == 1)
                flag_control = 1;//collect
            else if(flag == 2)
                flag_control = 2;//point to point
            else if(flag == 3)
                flag_control = 3;//apf
            else if(flag == 4)
                flag_control = 9;//stop
            else if(flag == 5)
                flag_control = 10;//keep
            else if(flag == 6)
                flag_control = 4;//tracking
            else if(flag == 7)
                flag_control = 5;//MPC-CBF-ST
            else if(flag == 8)
                flag_control = 6;// velocity tracking
        //}

        //to do
        switch (flag_control)
        {
        case 0:
            //warm up
            if(delta_v < cruise_speed)
                delta_v ++;
            else if(delta_v >= cruise_speed)
            {
                delta_v = cruise_speed;
                //flag = 7;//表明已匀速前进，MPC-CBF-ST
                flag = 8; // 速度跟踪测试
//                RX=x+4.1;
//                RY=y;
                //psi_d = Rov_psi + 45;//相对偏差角，好处是可以设置delta的初始值
            }
            cmd[26] = 128 - delta_v;
            cmd[27] = 128 - delta_v;
            break;
        case 1:
            //collect
            //注意rsv_data[]是serverwidget虚拟端得到的目标坐标信息，测试用。
            if(flag_save == 0)//test_mod
            {
                flag = 5;
                break;
            }

            if((rsv_data[0]==1)&&(rsv_data[1]==2)&&(rsv_data[16]==3)&&(rsv_data[17]==4))//receive right data
            {
                if((rsv_data[2]==0)&&(rsv_data[3]==0))//target not found
                {
                    flag = 2;//turn to navigation
                    break;
                }
                else if((rsv_data[2]==1)&&(rsv_data[3]==1))//target found
                {
                    //receive img_data
                    int left_center_x = (int)(rsv_data[4])*100 + (int)(rsv_data[5])*10 + (int)(rsv_data[6]);
                    int left_center_y = (int)(rsv_data[7])*100 + (int)(rsv_data[8])*10 + (int)(rsv_data[9]);
                    int right_center_x = (int)(rsv_data[10])*100 + (int)(rsv_data[11])*10 + (int)(rsv_data[12]);
                    int right_center_y = (int)(rsv_data[13])*100 + (int)(rsv_data[14])*10 + (int)(rsv_data[15]);
                    //stereo method

                    //turn_control
                }
            }
            break;
        case 2:
            //巡点导航
            psi_solve();
            if(num_i <= num_end)
            {
                if(abs(delta_psi) >= psi_def && delta_d > dis_def_navigate)// 转向控制调参 delta_psi取0只进入第一个if，实际导航可取2-3°
                {
                    //PID();
                    SMC();
                }
                else if(delta_d > dis_def_navigate)//(-delta_psi,delta_psi) 且 未到达终点
                {
                    cmd[26] = 128 - cruise_speed;
                    cmd[27] = 128 - cruise_speed;
                }
                else if(delta_d <= dis_def_navigate)//(-delta_psi,delta_psi) 且 到达终点
                {
                    if(num_i == num_end )
                        flag = 4;// 停转
                    num_i++;
                }
            }
            break;
        case 3:
            //APF-LOS
            // LOS法确定跟踪点 + 转向控制
            afa_k = atan2(p_next_x - p_x, p_next_y - p_y);
            s = (y - p_y) * cos(afa_k) + (x - p_x ) * sin(afa_k);
            delta_e = -(y - p_y) * sin(afa_k) + (x - p_x) * cos(afa_k);
            dx = delta_e * sin(afa_k);
            dy = delta_e * cos(afa_k);
            if (k == simple_len - 1)// pk为终点时
            {
                //x_los = goalX;// 终点直接作为LOS目标点
                //y_los = goalY;
                dist_goal = sqrt(pow(goalX - x, 2)+pow(goalY - y, 2));
                qDebug()<<"dist_goal= "<<dist_goal;
                if (dist_goal <= dist_IOU && flag2 == 0)// 到达到终点
                {
                    qDebug() << "Target reached!" << endl;
                    // 机器人刹车
                    flag = 4;//停止态
                    flag2++;
                    // 求解delta_psi
                    psi_solve();
                    // 将最后一段路径点保存到 path_final
                    int last_len = final_len;
                    int j = 0;
                    for (int i = last_len; i <= last_len + k; i++)
                    {
                        path_final[i][0] = path_simple[j][0];
                        path_final[i][1] = path_simple[j][1];
                        final_len++;
                        j++;
                    }
                }
                else if(dist_goal > dist_IOU && flag2 == 0)// 还没到终点
                {
                    // 求解LOS点
                    LOS_Solve();
                    // 求解delta_psi
                    psi_solve();
                    // 推进器转向控制
                    if(abs(delta_psi) >= psi_def)//
                    {
                        //PID();
                        SMC();
                    }
                    else //(-delta_psi,delta_psi) 且 未到达终点
                    {
                        cmd[26] = 128 - cruise_speed;
                        cmd[27] = 128 - cruise_speed;
                    }

                }
                else if(flag2 == 1)// 到终点后保持静止
                {
                   // 机器人刹车
//                    if(i < 10)//制动1s后，停转
//                    {
//                        i ++;
//                        cmd[26] = 128 + 15 + cruise_speed;
//                        cmd[27] = 128 + 15 + cruise_speed;
//                    }
//                    else
//                    {
//                        flag = 4;//停转
//                    }
                    flag = 4;//停转
                }
            }
            else if(flag2 == 0)// pk不为终点时
            {
                if (dist_D - s <= 2 * Lpp)
                {
                    k++; // 更新下一条线段
                    if (k < simple_len - 1)
                    {
                        p_x = path_simple[k][0];
                        p_y = path_simple[k][1];
                        p_next_x = path_simple[k + 1][0];
                        p_next_y = path_simple[k + 1][1];
                    }
                }
                // 求解LOS点
                LOS_Solve();
                // 求解delta_psi
                psi_solve();
                // 推进器转向控制
                if(abs(delta_psi) >= psi_def)//
                {
                    //PID();
                    SMC();
                }
                else //(-delta_psi,delta_psi) 且 未到达终点
                {
                    cmd[26] = 128 - cruise_speed;
                    cmd[27] = 128 - cruise_speed;
                }
            }
            break;
        case 4:
            //轨迹跟踪
            //(5,-5) R=4
            x_los = RX - 4 * cos(w*timestep*d_t);
            y_los = RY - 4 * sin(w*timestep*d_t);
            timestep++;

            delta_d_last = delta_d;
            delta_d = sqrt(pow(x_los-x,2) + pow(y_los-y,2));//De
            delta_d_diff = (delta_d - delta_d_last)/d_t;//De'
            // 求解delta_psi
            psi_solve();
            SMC(); //滑模控制
            break;
        case 5:
            // MPC-CBF-ST
            // matlab接收成功再控制
            if(matlab_rsv_flag == 1)
            {
                ST();
                matlab_rsv_flag = 0;
            }
            break;
        case 6:
            // 角速度跟踪 (仅当参考角速度 ref_r != 0 时生效)
            {
                // 增量式 PID & SMC 切换，使用全局参数。误差单位: 度/秒。
                static double e_prev1 = 0.0; // e_{k-1}
                static double e_prev2 = 0.0; // e_{k-2}
                static double u_prev = 0.0;  // 上一控制输出 tau
                static double smc_S_last = 0.0; // 上一 S 值

                // 角速度跟踪启用条件：幅值 > 0 (而不是 ref_r != 0)
                if(ref_r_amp > 0.0) {
                    // 原始误差(rad/s) -> 度/秒
                    double r_error_deg = (ref_r - Rov_r) * 180.0 / M_PI;

                    if(g_r_track_mode == 0) { // PID 增量式
                        // 增量公式: Δu = Kp*(e_k - e_{k-1}) + Ki*Ts*e_k + Kd*(e_k - 2e_{k-1} + e_{k-2})/Ts
                        double du = g_pid_kp_r * (r_error_deg - e_prev1)
                                   + g_pid_ki_r * Ts * r_error_deg
                                   + g_pid_kd_r * (r_error_deg - 2*e_prev1 + e_prev2) / Ts;
                        double u_lin = u_prev + du; // 这是“虚拟的线性控制量”

                        // 幅度限制
                        if(u_lin > 22) u_lin = 22; else if(u_lin < -22) u_lin = -22;

                        // 死区补偿
                        double u_cmd;
                        if (u_lin > 0)
                            u_cmd = u_lin + 18;       // 反补偿正向死区
                        else if (u_lin < 0)
                            u_cmd = u_lin - 18;       // 反补偿负向死区
                        else
                            u_cmd = 0;                // 正好停转
                        
                        // 硬件限幅（[-40, 40]）
                        if (u_cmd > 40) u_cmd = 40;
                        else if (u_cmd < -40) u_cmd = -40;

                        // 生成指令
                        int tau_cmd = (u_cmd >= 0)? static_cast<int>(u_cmd + 0.5) : static_cast<int>(u_cmd - 0.5);
                        cmd[26] = 128 + tau_cmd;
                        cmd[27] = 128 - tau_cmd;

                        // 更新历史
                        e_prev2 = e_prev1;
                        e_prev1 = r_error_deg;
                        u_prev = u_lin;
                    }
                    else if(g_r_track_mode == 1) { // SMC
                        double de = (r_error_deg - e_prev1) / Ts; // 误差变化率(度/秒^2)
                        double S_r = g_smc_lambda_r * r_error_deg + de;
                        double satS;
                        if(S_r > g_smc_sigma_r) satS = 1.0; else if(S_r < -g_smc_sigma_r) satS = -1.0; else satS = S_r / g_smc_sigma_r;
                        // 使用度制误差近似控制, 简化项保持跟踪与阻尼。注意 Rov_r 仍是 rad/s 转换为度/秒以一致使用.
                        double Rov_r_deg = Rov_r * 180.0 / M_PI;
                        double tau = g_smc_C0_r * satS + m_r * (g_smc_lambda_r * de - g_smc_lambda_r * Rov_r_deg - (d_r / m_r) * Rov_r_deg);
                        if(tau > 40) tau = 40; else if(tau < -40) tau = -40;
                        if(tau > 0 && tau < 10) tau = 10; else if(tau < 0 && tau > -10) tau = -10;
                        int tau_cmd = (tau >= 0)? static_cast<int>(tau + 0.5) : static_cast<int>(tau - 0.5);
                        cmd[26] = 128 + tau_cmd;
                        cmd[27] = 128 - tau_cmd;
                        smc_S_last = S_r;
                        // 更新误差历史（用于下一步 de 计算）
                        e_prev2 = e_prev1;
                        e_prev1 = r_error_deg;
                    }
                }
            }
            break;

        case 7:
            cmd[28] = 128;//前推静止
            cmd[29] = 128;
            break;
        case 8:
            // 线速度 u, v 跟踪
            // u 对应 cmd[26], cmd[27]
            // v 对应 cmd[28], cmd[29]
            {
                // 增量式 PID: Δu = Kp*(e_k-e_{k-1}) + Ki*Ts*e_k + Kd*(e_k - 2e_{k-1} + e_{k-2})/Ts
                // 激活条件：对应幅值 > 0 (正弦或常值跟踪开启)，否则不改变推进器指令
                static double u_e_prev1 = 0.0, u_e_prev2 = 0.0, u_cmd_prev = 0.0;
                static double v_e_prev1 = 0.0, v_e_prev2 = 0.0, v_cmd_prev = 0.0;

                // 可调增益（后续可移入全局参数对话框）
                const double Kp_u = 60.0, Ki_u = 0.0, Kd_u = 8.0; // u 轴 PID
                const double Kp_v = 60.0, Ki_v = 0.0, Kd_v = 8.0; // v 轴 PID
                const double max_speed = 22.0; // 推进器幅度限幅
                const double dead_zone = 18.0;  // 推进器最低有效值

                // 误差：参考 - 当前
                double u_error = ref_u - Rov_u; // m/s
                double v_error = ref_v - Rov_v;   // 侧向速度（假设 v_y 为船体坐标系侧向速度）

                // -------- u 轴跟踪 --------
                if(ref_u_amp > 0.0 || std::abs(ref_u_base) > 1e-9) { // 有幅值或基值非零
                    double du_inc = Kp_u * (u_error - u_e_prev1)
                                  + Ki_u * Ts * u_error
                                  + Kd_u * (u_error - 2*u_e_prev1 + u_e_prev2) / Ts;
                    double u_lin = u_cmd_prev + du_inc; // 未限幅指令
                    // 限幅
                    if(u_lin > max_speed) u_lin = max_speed; else if(u_lin < -max_speed) u_lin = -max_speed;
                    // 死区补偿
                    double u_cmd;
                    if(u_lin > 0) 
                        u_cmd = u_lin + dead_zone; 
                    else if(u_lin < 0) 
                        u_cmd = u_lin - dead_zone;
                    else
                        u_cmd = 0;
                    // 映射到后部推进器 (对称，不含转向)
                    int thrust = (u_cmd >= 0)? static_cast<int>(u_cmd + 0.5) : static_cast<int>(u_cmd - 0.5);
                    cmd[26] = 128 - thrust;
                    cmd[27] = 128 - thrust;
                    // 更新历史
                    u_e_prev2 = u_e_prev1; 
                    u_e_prev1 = u_error; 
                    u_cmd_prev = u_cmd;
                }

                // -------- v 轴跟踪 --------
                if(ref_v_amp > 0.0 || std::abs(ref_v_base) > 1e-9) {
                    double dv_inc = Kp_v * (v_error - v_e_prev1)
                                  + Ki_v * Ts * v_error
                                  + Kd_v * (v_error - 2*v_e_prev1 + v_e_prev2) / Ts;
                    double v_lin = v_cmd_prev + dv_inc;
                    if(v_lin > max_speed) v_lin = max_speed; else if(v_lin < -max_speed) v_lin = -max_speed;
                    double v_cmd;
                    if(v_lin > 0) 
                        v_cmd = v_lin + dead_zone; 
                    else if(v_lin < 0) 
                        v_cmd = v_lin - dead_zone;
                    else
                        v_cmd = 0;
                    int thrust_v = (v_cmd >= 0)? static_cast<int>(v_cmd + 0.5) : static_cast<int>(v_cmd - 0.5);
                    cmd[28] = 128 - thrust_v;
                    cmd[29] = 128 - thrust_v;
                    v_e_prev2 = v_e_prev1; 
                    v_e_prev1 = v_error; 
                    v_cmd_prev = v_cmd;
                }
            }
            break;

        case 9:
            cmd[26] = 128;//推进器停转
            cmd[27] = 128;
            cmd[28] = 128;
            cmd[29] = 128;
            break;
        case 10://过渡态，维持原样----等待。
            break;
        }

        // 更新last步长的数据
//        Rov_psi_last = Rov_psi;
//        Rov_psi_diff_last = Rov_psi_diff;
        delta_psi_last = delta_psi;
        psi_d_last = psi_d;
        psi_d_diff_last = psi_d_diff;
        tau_n_last = tau_n;

        mpc_u_e_last = mpc_u_e;
        mpc_r_e_last =  mpc_r_e;
        psi_e_last = psi_e;

        //输出日志
        qDebug()<<"flag_control = "<<flag_control;
        qDebug()<<cmd[26];
        qDebug()<<cmd[27];
        //qDebug()<<"num_i = "<<num_i;//巡点序号

        QString y1 = QString::number(delta_psi,'f',1);
        QString y2 = QString::number(tau_n,'f',2);
        QString y3 = QString::number(Rov_psi,'f',2);//X轴逆时针正向0-pi
        QString y4 = QString::number(Rov_psi_diff,'f',2);
        QString y5 = QString::number(Rov_psi_diff_2,'f',2);

        // qDebug()<<"Rov_psi = "<<Rov_psi<<", Rov_psi_last = "<<Rov_psi_last<<endl;
        //写入文本文件
        out <<my_eso->z1<<" "<<my_eso->z2<<" "<<my_eso->z3<<" "<<x<<" "<<y<<" "<<y3<<" "<<y4<<" "<<y5<<" "<<speed_u<<" "<<motor_value<<" "<<Rov_u<<" "<<Rov_r<<" "<<mpc_u<<" "<<mpc_r<<" "<<mpc_u_e<<" "<<mpc_r_e<<" "<<delta_psi<<endl;
        timer_control->start(Ts*1000);//重启定时器,每100ms查看一次状态
    });

}

//析构函数
ControlThread::~ControlThread()
{
    flag = 0;
    delete timer_control;//关闭各定时器

    file1.close();
    file2.close();
    file3.close();
    qDebug() << ("文件已关闭");

//    delete timer_turn_left;
//    delete timer_turn_right;
//    delete timer_left;
//    delete timer_right;
//    delete timer_motor_run;
//    delete timer_motor_stop;
}

void ControlThread::sleep(unsigned int msec)
{
    //currnentTime 返回当前时间 用当前时间加上我们要延时的时间msec得到一个新的时刻
    QTime reachTime = QTime::currentTime().addMSecs(msec);
    //用while循环不断比对当前时间与我们设定的时间
    while(QTime::currentTime()<reachTime)
    {
        //如果当前的系统时间尚未达到我们设定的时刻，就让Qt的应用程序类执行默认的处理，
        //以使程序仍处于响应状态。一旦到达了我们设定的时刻，就跳出该循环，继续执行后面的语句。
        QApplication::processEvents(QEventLoop::AllEvents,100);
    }
}


void ControlThread::doProcessTimerStop()
{
//    flag_control = 9;//置为静止态.这样设置有问题：flag_control虽设为静止态的9，但若50ms没到、timer_control还未进入中断就被stop()了，则flag_control的设置不起作用，推进器依旧维持原状态。
    timer_control->stop();//先暂停各定时器
//    timer_turn_left->stop();
//    timer_turn_right->stop();
//    timer_left->stop();
//    timer_right->stop();
//    timer_motor_run->stop();
//    timer_motor_stop->stop();
    cmd[26] = 128;//再设置推进器停转
    cmd[27] = 128;
    cmd[28] = 128;
    cmd[29] = 128;

}


void ControlThread::doProcessControl()
{
    delta_v = 0;//每次进入自动模式，从状态0开始遍历
    flag = 0;
    flag_control = 0;
    flag_save = 0;
    timer_control->start(Ts*1000);//开启timer_control定时器
    i=1;
    //emit SigToMove_Forward();
}

//计算角度 0 - 360
float ControlThread::psi(float x,float y)
{
    float angle = 0;
    if(x == 0 || y == 0)
    {
        angle = 0;
    }
    else
    {
        angle = atan2(y,x);//第一象限
    }
    angle = angle * 180 / M_PI;//转为角度

    //转为x轴正向开始，逆时针0-180
    if (angle >= 180)
        angle = 180;
    else if(angle <= -180)
        angle = -180;

    return angle;
}

//饱和函数
float ControlThread::sat(double S, double sigma)
{
    float s;
    if (S > sigma)
        s = 1;
    else if (S < -sigma)
        s = -1;
    else
        s = S / sigma;
    return s;

}

//偏航角更新
void ControlThread::psi_solve()
{
    //求psi_d
    float d_x = x_los - x;
    float d_y = y_los - y;
//    float psi_los = psi(d_x, d_y);// 转为角度制，方便显示以及设阈值比较
    if(d_x != 0 || d_y != 0)
    {
        psi_d = atan2(d_y,d_x) * 180 / M_PI; //转为角度
    }
    else
    {
        psi_d = Rov_psi; //
    }
    qDebug()<<d_x<<" "<<d_y<<""<<psi_d<<endl;

    // 更新姿态角
    if(abs(psi_d - Rov_psi)<=180)
        delta_psi = psi_d - Rov_psi;
    else if((psi_d - Rov_psi) > 180)
        delta_psi = psi_d - Rov_psi - 360;//校正临界突变，限制在[-180,180]
    else if((psi_d - Rov_psi) < -180)
        delta_psi = psi_d - Rov_psi + 360;
    //限幅
    if(delta_psi >= 40)
        delta_psi = 40;
    else if(delta_psi <=-40)
        delta_psi = -40;

//    // 更新微分量, 转为rad/s 取两次平均值
//    float psi_diff = 0;
//    if (abs(Rov_psi - Rov_psi_last) < 180)
//        psi_diff = (Rov_psi - Rov_psi_last) * (M_PI / 180) / Ts;// r = psi'
//    else if(Rov_psi - Rov_psi_last >= 180)
//        psi_diff = (-360 + Rov_psi - Rov_psi_last) * (M_PI / 180) / Ts;// r = psi'
//    else if(Rov_psi - Rov_psi_last <= -180)
//        psi_diff = (360 + Rov_psi - Rov_psi_last) * (M_PI / 180) / Ts;// r = psi'

//    Rov_psi_diff = psi_diff;
//    qDebug()<<"Rov_psi_diff = "<<Rov_psi_diff<<endl;
//    //qDebug()<<"filter_buf[] = "<<filter_buf[0]<<" "<<filter_buf[1]<<" "<<filter_buf[2]<<" "<<filter_buf[3]<<" "<<filter_buf[5]<<endl;

//    Rov_psi_diff_2 = (Rov_psi_diff - Rov_psi_diff_last) * (M_PI / 180) / Ts;// r'
    psi_d_diff = (psi_d - psi_d_last) * (M_PI / 180) / Ts;// psi_d'
    psi_d_diff_2 = (psi_d_diff - psi_d_diff_last) * (M_PI / 180) / Ts;// psi_d''
    delta_psi_diff = psi_d_diff - Rov_psi_diff;// delta_psi'
}
// PID控制
void ControlThread::PID()
{
    int tau = 0;
    tau_n = kp *delta_psi + kd *(delta_psi - delta_psi_last);//pd控制
    if(tau_n >= 40)//幅度上限
        tau_n = 40;
    else if(tau_n <= -40)
        tau_n = -40;
    else if(tau_n >0 && tau_n <=2)//幅度下限
        tau_n = 2;
    else if(tau_n < 0 && tau_n >= -2)
        tau_n = -2;
    if(tau_n >= 0)
        tau = (int)(tau_n + 0.5);//四舍五入取整
    else
        tau = (int)(tau_n - 0.5);
    cmd[26] = 128 - tau;
    cmd[27] = 128 + tau;
}
// ST速度跟踪控制
void ControlThread::ST()
{
    //处理速度偏差，给pid使用
    // mpc_r = -0.2;
    mpc_u_e = mpc_u - Rov_u;
    if(abs(my_eso->z2 - Rov_r) > 0)
    {
        mpc_r_e = mpc_r - Rov_r;
    }
    else
    {
        mpc_r_e = mpc_r - my_eso->z2;//当ESO收敛时，采用估计速度
    }
    qDebug()<<"mpc_r_e = : "<<mpc_r_e;

    // ST控制
//    float ku = 0.5; //dx和推力F的比例系数（近似线性）
//    float kr = 1;
//    //去死区
//    if(mpc_tau_r > 0)
//        motor_value = kr * mpc_tau_r + 10;//电机死区是(-10,10)
//    else if(mpc_tau_r < 0)
//        motor_value = kr * mpc_tau_r - 10;
//    else
//        motor_value = 0;//推进器停转
//    if(mpc_tau_u > 0)
//    {
//        // speed_u = ku * mpc_tau_u + 10;//电机死区是(-10,10)
//        speed_u = 15;
//    }
//    else if(mpc_tau_u < 0)
//        speed_u = ku * mpc_tau_u - 10;
//    else
//        speed_u = 0;//推进器停转

    // PID方法，调试
    speed_u = 11;
    kp = 80;
    kd = 8;
    ki = 0;
    //speed_u = kp* mpc_u_e + kd*(mpc_u_e - mpc_u_e_last);
    //积分限幅
    float max = 10;
    if (abs(sum) < max)
        sum = sum + mpc_r_e;
    else if(sum >= max)
        sum = max;
    else if(sum <= -max)
        sum = -max;
    float tau_r = 0;
    tau_r = kp* mpc_r_e - kd*(mpc_r_e - mpc_r_e_last) + ki * sum;
    //角度偏差
//    psi_e = 90 - Rov_psi;
//    if(abs(psi_e) > 3)
//        tau_r = kp* mpc_r_e - kd*(mpc_r_e - mpc_r_e_last) + ki * sum;
//    else
//    {
//        tau_r = 0;
//        speed_u = 11;
//    }
    //tau_r = 0.2* psi_e - 1*(psi_e - psi_e_last);
    //去死区
    if(tau_r > 0)
        motor_value = tau_r + 10;//电机死区是(-10,10)
    else if(tau_r < 0)
        motor_value = tau_r - 10;
    else
        motor_value = 0;//推进器停转

    //限幅
    if(speed_u >= 60)
        speed_u = 60;
    else if(speed_u <= -60)
        speed_u = -60;
    if(motor_value >= 50)
        motor_value = 50;
    else if(motor_value <= -50)
        motor_value = -50;
    //motor_value = 30;
    //speed_u = 0;
    cmd[26] = (128 - speed_u) - motor_value;//左后
    cmd[27] = (128 - speed_u) + motor_value;//右后
}

// SMC控制
void ControlThread::SMC()
{
    float kf = 0.1;//dx和推力F的比例系数（近似线性）
    // 滑模面计算
    S = lambda_0 * delta_psi *(M_PI/180) + delta_psi_diff;// delta_psi转为弧度计算
    float sgn_S = sat(S, sigma);
    // 扭矩计算
    float r = Rov_psi_diff;
    if(r == 0)
        tau_n = tau_n_last;
    else
    {
        tau_n = C0 * sgn_S + m_r * (lambda_0 * psi_d_diff + psi_d_diff_2 - r * d_r / m_r - lambda_0 * r);// u*v*(m_u - m_v)/d_r这一项没写
        //eso-smc
        //tau_n = C0 * sgn_S + m_r * (lambda_0* delta_psi_diff + psi_d_diff_2 + 1 * my_eso->z3);
    }
    motor_value = kf * tau_n/(2*l);
    int max = 40;
    if(motor_value >= max)//幅度上限
        motor_value = max;
    else if(motor_value <= -max)
        motor_value = -max;
//    else if(motor_value >0 && motor_value <=2)//幅度下限
//        motor_value = 2;
//    else if(motor_value < 0 && motor_value >= -2)
//        motor_value = -2;
    speed_u = 25;
    cmd[26] = (128 - speed_u) + motor_value;
    cmd[27] = (128 - speed_u) - motor_value;
}

void ControlThread::APF_Generate()
{
    // 重置路径点向量
    pathX_last = pathX;
    pathY_last = pathY;
    pathX_last_last = pathX;
    pathY_last_last = pathY;
    // 重置合力向量
    uint_F_X = 0.0;
    uint_F_Y = 0.0;
    uint_F_X_last = 0.0;
    uint_F_Y_last = 0.0;
    // 重置路径长度
    real_len = 0;
    simple_len = 0;

    for (int i = 0; i <= iter_len; i++)
    {
        pathX_last_last = pathX_last;//更新上上步的路径位置
        pathY_last_last = pathY_last;
        pathX_last = pathX;//更新上一步的位置
        pathY_last = pathY;
        uint_F_X_last = uint_F_X;//更新上一步的合力方向
        uint_F_Y_last = uint_F_Y;

        //std::cout<< "[iteration:" << i << "]   Path position : (" << pathX << ", " << pathY << ")" << endl;//打印此时的坐标

        // 与目标距离小于0.5米时，视作到达
        if (sqrt(pow(goalX - pathX, 2) + pow(goalY - pathY, 2)) <= 0.5)
        {
            qDebug() << "Goal reached!" << endl;
            real_len = i;// 路径实际点数-1
            qDebug() << "real_len:" << i << endl;
            break;
        }
        // 路径遇到局部最优点时，会在两点之间抖动
        if (pathX == pathX_last_last && i != 0)
        {
            qDebug() << "Local optimal solution reached!" << endl;
            pathX += 0.5;//手动跳出局部最优点
            pathY -= 0.5;
            break;
        }

        APF_Calculate();//否则，继续生成路径

        //存储路径向量
        path[i][0] = pathX;
        path[i][1] = pathY;
        qDebug()<<"("<<pathX <<","<<pathY<<")"<<endl;
        //out2 << path[i][0] << " " << path[i][1] << endl;

    }
    // 均值滤波
    for (int i = 1; i < real_len - 1; i += 1)
    {
        path[i][0] = (path[i-1][0] + path[i][0]) / 2;//2均值滤波
        path[i][1] = (path[i-1][1] + path[i][1]) / 2;
    }
    // 选取关键路径点
    // 法1 直接每间隔10个点取一个点
    //int j = 0;
    //for (int i = 0; i <= real_len; i += 10)
    //{
    //    if (i > (real_len - 10))
    //    {
    //        path_simple[j][0] = path[real_len-1][0];
    //        path_simple[j][1] = path[real_len-1][1];
    //    }
    //    else
    //    {
    //        path_simple[j][0] = path[i][0];
    //        path_simple[j][1] = path[i][1];
    //    }
    //    j ++;
    //}
    //simple_len = j;
    //cout << "simple_len:" << simple_len << endl;
    //
    // 法2 相邻点间隔5米（终点为p to p，除外）
    int i = 0, k = 0;
    for (int j = 1; j <= real_len; j++)
    {
        if (j == real_len)
        {
            path_simple[k][0] = goalX;// 终点则直接赋值
            path_simple[k][1] = goalY;
            k++;
        }
        else if (sqrt(pow(path[i][0] - path[j][0], 2) + pow(path[i][1] - path[j][1], 2)) >= dist_D)// i,j间距大于dist_D时
        {
            path_simple[k][0] = path[i][0];
            path_simple[k][1] = path[i][1];
            i = j ;//仅距离符合要求时，i才增加
            k++;
        }
    }
    simple_len = k;// 路径采样点数量
//    std::cout << "simple_len:" << simple_len << endl;
//    std::cout << "obs_new_count: " << obs_new_count << endl;
//    std::cout << "obs_count: " << obs_count << endl;
//    std::cout << "obstacle[0][0]: " << obstacle[0][0] << endl;
//    std::cout << "obstacle[0][1]: " << obstacle[0][1] << endl;
    return;
}

void ControlThread::APF_Calculate()
{
    const double ka = 1.0;//引力系数
    const double kr = 4.5;//斥力系数

    // 计算引力
    double dist_g = sqrt(pow(goalX - pathX, 2) + pow(goalY - pathY, 2));//目标距离
    double Fatt_X = ka * (goalX - pathX);//引力的x,y分量
    double Fatt_Y = ka * (goalY - pathY);

    //计算斥力
    double Frep1_X = 0;
    double Frep1_Y = 0;
    double Frep2_X = 0;
    double Frep2_Y = 0;
    double Frep_X = 0;
    double Frep_Y = 0;

    //对每个障碍物分别计算斥力，再累加
    for (int i = 0;i < obs_count;i++)
    {
        // 读取一个障碍物坐标
        obstacleX = obstacle[i][0];
        obstacleY = obstacle[i][1];
        double  dist_o = sqrt(pow(obstacleX - pathX, 2) + pow(obstacleY - pathY, 2));// 计算距离

        if (dist_o < rou_0) //在半径内部时，生成斥力
        {
            //采用改进的斥力模型，阶次n = 2
            Frep1_X = kr * (1 / dist_o - 1 / rou_0) * pow(dist_g, 2) * (pathX - obstacleX) / pow(dist_o, 3);//斥力1，由障碍物指向机器人
            Frep1_Y = kr * (1 / dist_o - 1 / rou_0) * pow(dist_g, 2) * (pathY - obstacleY) / pow(dist_o, 3);
            Frep2_X = kr * pow((1 / dist_o - 1 / rou_0), 2) * (goalX - pathX);//斥力2，由机器人指向目标物（与引力同向）
            Frep2_Y = kr * pow((1 / dist_o - 1 / rou_0), 2) * (goalY - pathY);
        }
        Frep_X += Frep1_X + Frep2_X;//累加，计算总斥力
        Frep_Y += Frep2_Y + Frep2_Y;
    }

    // 计算合力
    double F_X = Fatt_X + Frep_X;
    double F_Y = Fatt_Y + Frep_Y;
    uint_F_X = F_X / sqrt(pow(F_X, 2) + pow(F_Y, 2));//得到单位合力的方向向量
    uint_F_Y = F_Y / sqrt(pow(F_X, 2) + pow(F_Y, 2));

    //若与上次力的方向夹角大于20度，且不是初始状态，则取两次力的中间方向
    if (abs(atan(uint_F_Y / uint_F_X) * 180 / M_PI - atan(uint_F_Y_last / uint_F_X_last) * 180 / M_PI) >= 20 && pathX_last != 0)
    {
        uint_F_X = (uint_F_X + uint_F_X_last) / 2;
        uint_F_Y = (uint_F_Y + uint_F_Y_last) / 2;
    }

    pathX += path_len * uint_F_X;//更新当前路径位置
    pathY += path_len * uint_F_Y;
}

void ControlThread::LOS_Solve()
{
    float delta_x = p_next_x - p_x;
    float delta_y = p_next_y - p_y;
    d = delta_y / delta_x;
    e = p_x;
    f = p_y;
    g = f - d * e;
    a = 1 + pow(d, 2);
    b = 2 * (d*g - d*y -x);
    c = pow(x, 2) + pow(y, 2) + pow(g, 2) - 2*g*y - pow(R, 2);
    if (delta_x == 0)
    {
        x_los = p_x;
        if (delta_y > 0)
            y_los = y + sqrt(pow(R, 2) - pow(x_los - x, 2));
        else
            y_los = y - sqrt(pow(R, 2) - pow(x_los - x, 2));
    }
    else if (delta_x > 0)
    {
        x_los = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
        y_los = d * (x_los - p_x) + p_y;
    }
    else
    {
        x_los = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
        y_los = d * (x_los - p_x) + p_y;
    }
}

