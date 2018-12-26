module easy_maze
(
	clk_in,
	rst,
	key,
	map,
	row,
	col_red,
	col_green,
	cat,
	seg_led,
	pause
);
input clk_in;//ʱ������
input rst;//��λ�ź�
input [3:0] key;//��������
input map;//up1,down0,default0
input pause;
output [7:0]row,col_red,col_green;//�������
output [7:0]cat,seg_led;//�������ʾ

wire time_judge;//�ж�ʱ��� =1��ʱ =0δ��ʱ
reg win_judge;// =1ʤ�� =0δʤ��
wire [7:0] movecnt;//�Ʋ���������
wire [63:0] pos;//λ���ź�
wire map_judge;

always@(posedge clk_in or posedge rst)
begin
	if(rst)
		win_judge = 1'b0;
	else if(!map)
		win_judge = pos[15];
	else
		win_judge = pos[58];
end

time_move_cnt u1
(
	.clk_in(clk_in),
	.rst(rst),
	.seg_data7(movecnt[7:4]),
	.seg_data6(movecnt[3:0]),
	.seg_led(seg_led),.cat(cat),
	.win_judge(win_judge),
	.flag(time_judge)
);
direction_judge u2
(
	.clk_in(clk_in),
	.rst(rst),
	.flag(time_judge),
	.win_judge(win_judge),
	.key(key),.pos(pos),
	.movecnt(movecnt),
	.map(map)
);
led88 u3
(
	.clk_in(clk_in),
	.rst(rst),.pos(pos),
	.row(row),
	.col_red(col_red),
	.col_green(col_green),
	.time_judge(time_judge),
	.win_judge(win_judge),
	.map(map),
	.pause(pause)
);
	
endmodule


//�Թ�������ʾģ��
module led88
(
	clk_in,
	rst,
	pos,
	row,
	col_red,
	col_green,
	time_judge,
	win_judge,
	map,
	pause
);
input clk_in,rst;
input time_judge,win_judge;
input  [63:0] pos;
input map;
input pause;
output reg [7:0] row,col_red,col_green;
reg [2:0] cnt;
wire clk_1k;

defparam divd_1k.divdN=50000,divd_1k.divdWIDTH=16;//�ض����Ƶ��ģ�����
divd_frequency divd_1k(.rst(rst),.clk_in(clk_in),.clk_out(clk_1k));//��Ƶ�� ��Ƶ1kHz

always@(posedge clk_1k or posedge rst)
begin
	if(rst)//��λ�ж�
		cnt<=3'b000;
	else
		cnt<=cnt+1'b1;
end


always@(cnt or rst)
begin
	if(rst)
		begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b00000000;col_green=pos[63:56];end
			3'b001:begin row=8'b10111111;col_red=8'b00000000;col_green=pos[55:48];end
			3'b010:begin row=8'b11011111;col_red=8'b00000000;col_green=pos[47:40];end
			3'b011:begin row=8'b11101111;col_red=8'b00000000;col_green=pos[39:32];end
			3'b100:begin row=8'b11110111;col_red=8'b00000000;col_green=pos[31:24];end
			3'b101:begin row=8'b11111011;col_red=8'b00000000;col_green=pos[23:16];end
			3'b110:begin row=8'b11111101;col_red=8'b00000000;col_green=pos[15: 8];end
			3'b111:begin row=8'b11111110;col_red=8'b00000000;col_green=pos[ 7: 0];end
			default:;
			endcase
		end
	else if( (!time_judge) & (!win_judge) & (!map))//map0������
	begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b01111111;col_green=pos[63:56];end
			3'b001:begin row=8'b10111111;col_red=8'b01000001;col_green=pos[55:48];end
			3'b010:begin row=8'b11011111;col_red=8'b01011101;col_green=pos[47:40];end
			3'b011:begin row=8'b11101111;col_red=8'b01010101;col_green=pos[39:32];end
			3'b100:begin row=8'b11110111;col_red=8'b01010101;col_green=pos[31:24];end
			3'b101:begin row=8'b11111011;col_red=8'b11000101;col_green=pos[23:16];end
			3'b110:begin row=8'b11111101;col_red=8'b00011100;col_green=pos[15: 8];end
			3'b111:begin row=8'b11111110;col_red=8'b11111111;col_green=pos[ 7: 0];end
			default:;
		endcase
	end
	else if( (!time_judge) & (!win_judge) & (map))
	begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b0001561010;col_green=pos[63:56];end
			3'b001:begin row=8'b10111111;col_red=8'b01111010;col_green=pos[55:48];end
			3'b010:begin row=8'b11011111;col_red=8'b01000011;col_green=pos[47:40];end
			3'b011:begin row=8'b11101111;col_red=8'b11011100;col_green=pos[39:32];end
			3'b100:begin row=8'b11110111;col_red=8'b10010101;col_green=pos[31:24];end
			3'b101:begin row=8'b11111011;col_red=8'b10110101;col_green=pos[23:16];end
			3'b110:begin row=8'b11111101;col_red=8'b10000001;col_green=pos[15: 8];end
			3'b111:begin row=8'b11111110;col_red=8'b11111111;col_green=pos[ 7: 0];end
			default:;
		endcase
	end
	else if( (~time_judge) & win_judge )//ʤ��
	begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b00000000;end
			3'b001:begin row=8'b10111111;col_red=8'b01100110;end
			3'b010:begin row=8'b11011111;col_red=8'b11111111;end
			3'b011:begin row=8'b11101111;col_red=8'b11111111;end
			3'b100:begin row=8'b11110111;col_red=8'b11111111;end
			3'b101:begin row=8'b11111011;col_red=8'b01111110;end
			3'b110:begin row=8'b11111101;col_red=8'b00111100;end
			3'b111:begin row=8'b11111110;col_red=8'b00011000;end
			default:;
		endcase
	end
	else if( time_judge & (~win_judge) )//ʧ��
	begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b00000000;end
			3'b001:begin row=8'b10111111;col_red=8'b01100110;end
			3'b010:begin row=8'b11011111;col_red=8'b11100111;end
			3'b011:begin row=8'b11101111;col_red=8'b11100111;end
			3'b100:begin row=8'b11110111;col_red=8'b11100111;end
			3'b101:begin row=8'b11111011;col_red=8'b01100110;end
			3'b110:begin row=8'b11111101;col_red=8'b00100100;end
			3'b111:begin row=8'b11111110;col_red=8'b00000000;end
			default:;
		endcase
	end
end

endmodule

//����ʱ �Ʋ���
module time_move_cnt(
	clk_in,
	rst,
	seg_data7,
	seg_data6,
	seg_led,
	cat,
	win_judge,
	flag
);
input clk_in,rst;
input win_judge;
input wire [3:0] seg_data7,seg_data6;//�Ʋ������ݴ��ݷ�����ƴ���Ĳ���
output  wire [7:0] seg_led;//�������ʾ��·�ź�
output  wire [7:0] cat;
output reg flag;//����ʱ��0ʱ����flagֹͣ���� ������Ϊ�ж���Ϸʧ�ܵ��ź� 1Ϊʧ��

wire clk_1;
reg [3:0] seg_data1,seg_data0;//����ʱ����

defparam divd_1.divdN=50000000,divd_1.divdWIDTH=28;//�ض����Ƶ��ģ�����
divd_frequency divd_1(.rst(rst),.clk_in(clk_in),.clk_out(clk_1));//��Ƶ�� ��Ƶ1Hz

always@(posedge clk_1 or posedge rst)//��ʱ������
begin
	if(rst)
	begin 
		seg_data1 <= 4'd3;
		seg_data0 <= 4'd0;
		flag <= 1'b0;
	end
	else if(!win_judge)
	begin
		if(flag)
			begin seg_data1 <= 4'd0;seg_data0 <= 4'd0;end
		else
		begin
			if(seg_data1 == 4'd0 && seg_data0 == 4'd0)
				flag <= 1'b1;
			else
			begin
				if(seg_data0 == 4'd0)//��λΪ0��
				begin
					seg_data1 <= seg_data1 - 1'b1;
					seg_data0 <= 4'd9;
				end
				else
					seg_data0 <= seg_data0 - 1'b1;
			end
		end	
	end
end

segment s1
(
	.clk_in(clk_in),
	.rst(rst),
	.seg_data7(seg_data7),
	.seg_data6(seg_data6),
	.seg_data1(seg_data1),
	.seg_data0(seg_data0),
	.seg_led(seg_led),
	.cat(cat)
);

endmodule

//�߶��������ʾ
module segment
(
	clk_in,
	rst,
	seg_data7,//�������ݣ�76Ϊ�Ʋ������� 10Ϊ����ʱ����
	seg_data6,
	seg_data1,
	seg_data0,
	seg_led,
	cat
);
input clk_in,rst;
input wire [3:0] seg_data7,seg_data6,seg_data1,seg_data0;
output reg [7:0] cat;//ɨ��͵�ƽ
output reg [7:0] seg_led;//���Ƹߵ�ƽ  ��λ����λ�����ң��ֱ�Ϊpabcdefg

reg [7:0] seg [9:0];// ����0~9ʮ�����ֵ���ʾ 10���Ĵ���seg ÿ���Ĵ���8λ

initial
begin
	seg[0] = 8'b01111110;
	seg[1] = 8'b00110000;
	seg[2] = 8'b01101101;
	seg[3] = 8'b01111001;
	seg[4] = 8'b00110011;
	seg[5] = 8'b01011011;
	seg[6] = 8'b01011111;
	seg[7] = 8'b01110000;
	seg[8] = 8'b01111111;
	seg[9] = 8'b01111011;
end

reg [1:0]cnt;
wire clk_1k;

defparam divd_1k.divdN=50000,divd_1k.divdWIDTH=16;//�ض����Ƶ��ģ�����
divd_frequency divd_1k(.rst(rst),.clk_in(clk_in),.clk_out(clk_1k));//��Ƶ�� ��Ƶ1kHz

always@(posedge clk_1k or posedge rst)
begin
	if(rst)
		cnt <= 2'b0;
	else
		cnt <= cnt + 1'b1;
end

always@(cnt)
begin
	case(cnt)
	2'b00:begin cat=8'b01111111;seg_led = seg[seg_data7];end
	2'b01:begin cat=8'b10111111;seg_led = seg[seg_data6];end
	2'b10:begin cat=8'b11111101;seg_led = seg[seg_data1];end
	2'b11:begin cat=8'b11111110;seg_led = seg[seg_data0];end
	default:;
	endcase
end
endmodule

//�������ģ��
module direction_judge
(
	clk_in,
	rst,
	flag,//Ϊ1ʱ ��Ϸʧ�� ֹͣ�Ʋ����ƶ�
	win_judge,
	key,
	pos,
	movecnt,//�Ʋ���������Ʋ�����seg_data����
	map
);
input clk_in,rst;
input flag,win_judge;
input [3:0] key;//��λ����λ�ֱ��ʾ��������
input map;
output reg [63:0] pos;
output reg [7:0]movecnt;//[7:4][3:0]��λ

wire [3:0] key_pulse;//�����߳�
reg [3:0] gold,goru;//��λ
reg judge;//judge=1ײǽ =0��ײǽ
reg [63:0] brick,temp;

always@(posedge clk_in)//����ǽ
begin
	case(map)
	1'b0:
		begin
			brick[63:56]=8'b01111111;
			brick[55:48]=8'b01000001;
			brick[47:40]=8'b01011101;
			brick[39:32]=8'b01010101;
			brick[31:24]=8'b01010101;
			brick[23:16]=8'b11000101;
			brick[15: 8]=8'b00011100;
			brick[ 7: 0]=8'b11111111;
		end
	1'b1:	
		begin
			brick[63:56]=8'b00011010;
			brick[55:48]=8'b01111010;
			brick[47:40]=8'b01000011;
			brick[39:32]=8'b11011100;
			brick[31:24]=8'b10010101;
			brick[23:16]=8'b10110101;
			brick[15: 8]=8'b10000001;
			brick[ 7: 0]=8'b11111111;
		end
	endcase
end

always@(posedge clk_in or posedge rst)
begin
	if(rst)
	case(map)
	1'b0:
	begin
		movecnt <= 8'h00;
		pos[63:56] <= 8'b00000000;
		pos[55:48] <= 8'b00000000;
		pos[47:40] <= 8'b00000000;
		pos[39:32] <= 8'b00000000;
		pos[31:24] <= 8'b00000000;
		pos[23:16] <= 8'b00000000;
		pos[15: 8] <= 8'b00000001;
		pos[ 7: 0] <= 8'b00000000;
	end
	1'b1:
	begin
		movecnt <= 8'h00;
		pos[63:56] <= 8'b00000000;
		pos[55:48] <= 8'b00000000;
		pos[47:40] <= 8'b00000000;
		pos[39:32] <= 8'b00000001;
		pos[31:24] <= 8'b00000000;
		pos[23:16] <= 8'b00000000;
		pos[15: 8] <= 8'b00000000;
		pos[ 7: 0] <= 8'b00000000;
	end
	endcase
	else if(!flag && !win_judge)//�ж���Ϸ�Ƿ�ʧ��
		if(key_pulse)
		begin
			case(key_pulse)
			4'b1000: begin gold = 4'd1; goru = 4'd0;end 
			4'b0100: begin goru = 4'd1; gold = 4'd0;end
			4'b0010: begin goru = 4'd8; gold = 4'd0;end
			4'b0001: begin gold = 4'd8; goru = 4'd0;end
			default:;
			endcase
			
			temp = pos >> gold << goru;//ײǽ�ж�
			if(temp & brick)
				judge = 1;
			else 
				judge = 0;
			
			if(!judge)//λ��
			begin
				pos = pos >> gold << goru;
				//�Ʋ���
				if(movecnt[3:0] == 4'd9)//��λ��9?
				begin
					movecnt[3:0] = 4'd0;
					if(movecnt[7:4] == 9)//ʮλ��9��
						movecnt[7:4] = 4'd0;
					else
						movecnt[7:4] = movecnt[7:4] + 1'b1;
				end
				else
					movecnt[3:0] = movecnt[3:0] + 1'b1;
			end
		end
end

debounce ul (.clk(clk_in),.rst(rst),.key(key[3]),.key_pulse(key_pulse[3]));//��������
debounce ur (.clk(clk_in),.rst(rst),.key(key[2]),.key_pulse(key_pulse[2]));
debounce uu (.clk(clk_in),.rst(rst),.key(key[1]),.key_pulse(key_pulse[1]));
debounce ud (.clk(clk_in),.rst(rst),.key(key[0]),.key_pulse(key_pulse[0]));

endmodule

//��Ƶ��ģ��
module divd_frequency(rst,clk_in,clk_out);

input rst,clk_in;
output reg clk_out;
parameter divdWIDTH=1;
parameter divdN=1;//��Ƶ����
reg [divdWIDTH:0] cnt;

always@(posedge clk_in or posedge rst)
	if(rst)
		begin
		cnt<=0;
		clk_out<=0;
		end
	else
		begin
		cnt<=cnt+1'b1;
		if(cnt==(divdN/2-1))
			begin
			cnt<=0;
			clk_out<=~clk_out;
			end
		end
endmodule

//��������ģ��
module debounce (clk,rst,key,key_pulse);
 
parameter       N  =  1;                      //Ҫ�����İ���������
 
input clk;
input rst;
input [N-1:0] key;                        //����İ���					
output [N-1:0] key_pulse;                  //������������������	
 
reg [N-1:0] key_rst_pre;                //����һ���Ĵ����ͱ����洢��һ������ʱ�İ���ֵ
reg [N-1:0] key_rst;                    //����һ���Ĵ����������洢��ǰʱ�̴����İ���ֵ
 
wire [N-1:0] key_edge;                   //��⵽�����ɸߵ��ͱ仯�ǲ���һ��������
 
//���÷�������ֵ�ص㣬������ʱ�Ӵ���ʱ����״̬�洢�������Ĵ���������
always @(posedge clk  or  posedge rst)
begin
	if (rst) begin
		key_rst <= {N{1'b0}};                //��ʼ��ʱ��key_rst��ֵȫΪ1��{}�б�ʾN��1
		key_rst_pre <= {N{1'b0}};
		end
	else begin
		key_rst <= key;                     //��һ��ʱ�������ش���֮��key��ֵ����key_rst,ͬʱkey_rst��ֵ����key_rst_pre
		key_rst_pre <= key_rst;             //��������ֵ���൱�ھ�������ʱ�Ӵ�����key_rst�洢���ǵ�ǰʱ��key��ֵ��key_rst_pre�洢����ǰһ��ʱ�ӵ�key��ֵ
		end    
end
 
assign  key_edge = (~key_rst_pre) & key_rst;//������ؼ�⡣��key��⵽������ʱ��key_edge����һ��ʱ�����ڵĸߵ�ƽ
 
reg	[19:0]	  cnt;                       //������ʱ���õļ�������ϵͳʱ��50MHz��Ҫ��ʱ20ms����ʱ�䣬������Ҫ20λ������     
 
//����20ms��ʱ������⵽key_edge��Ч�Ǽ��������㿪ʼ����
always @(posedge clk or posedge rst)
begin
	if(rst)
		cnt <= 20'h0;
	else if(key_edge)
		cnt <= 20'h0;
	else
		cnt <= cnt + 1'h1;
end  
 
reg [N-1:0] key_sec_pre;                //��ʱ�����ƽ�Ĵ�������
reg [N-1:0] key_sec;                    
 
 
//��ʱ����key���������״̬��Ͳ���һ��ʱ�ӵĸ����塣�������״̬�ǸߵĻ�˵��������Ч
always @(posedge clk  or  posedge rst)
begin
	if (rst) 
		key_sec <= {N{1'b0}};                
	else if (cnt==20'hfffff)
		key_sec <= key;  
end

always @(posedge clk  or  posedge rst)
begin
	if (rst)
		key_sec_pre <= {N{1'b0}};
	else                   
		key_sec_pre <= key_sec;             
end      
assign  key_pulse = (~key_sec_pre) & key_sec;     
 
endmodule