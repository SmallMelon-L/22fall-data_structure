#include <bits/stdc++.h>
#define PI 3.14
#define dis 50
#define sigma 25
#define beta 4.1
using namespace std;
struct point
{
	double x, y;
};
struct road
{
	int start, end;
	string level;
	int lv;
};
struct cmb
{
	int r;
	point p;
};
struct part
{
	int r;
	point b, e;
};
bool operator==(point a, point b)
{
	return a.x == b.x && a.y == b.y;
}
bool operator<(point a, point b)
{
	if(a.x == b.x)
	{
		return a.y < b.y;
	}
	else
	{
		return a.x < b.x;
	}
}
vector<part> v[2500][1500];
map<point, vector<cmb> > mp;
road e[121281];
point track[2100];
void read_map();
void read_track();
void XY(double&, double&);
void get_near(point, vector<point>&, vector<point>&, vector<point>&, vector<double>&, vector<int>&);
double dpl(point, point, point, point&);
double lacp(double);
double trans(point, point, point, point, point, point);
double route(point, point);
double dpp(point, point);
void read_double(double&);
template <class T>
inline bool scan_d(T &ret) {
    char c;
    int sgn;
    if (c = getchar(), c == EOF)
        return 0;  // EOF
    while (c != '-' && (c < '0' || c > '9')) c = getchar();
    sgn = (c == '-') ? -1 : 1;
    ret = (c == '-') ? 0 : (c - '0');
    while (c = getchar(), c >= '0' && c <= '9') ret = ret * 10 + (c - '0');
    ret *= sgn;
    return 1;
}
void read_double(double& x)
{
	char c;
	while((c = getchar()))
	{
		if(c >= '0' && c <= '9')
		{
			break;
		}
	}
	x = c - '0';
	while((c = getchar()) != '.')
	{
		x = x * 10 + c - '0';
	}
	double d = 0.1;
	while((c = getchar()))
	{
		if(c < '0' || c > '9')
		{
			break;
		}
		x += (c - '0') * d;
		d *= 0.1;
	}
	return;
}
void XY(double& y, double& x)
{
	y -= 30.6;
	x -= 121.003;
	y *= 111000;
	x *= 96126;
}
void read_map()
{
	int n;
	scanf("%d", &n);
	for(int i = 0; i != n; i++)
	{
		int c;
		point p0, p;
		scanf("%d%d%d", &c, &e[i].start, &e[i].end);
		getchar();
		while(getchar() != ' ');
		scanf("%d%d", &e[i].lv, &c);
		read_double(p0.y);
		read_double(p0.x);
		XY(p0.y, p0.x);
		for(int j = 1; j != c; j++)
		{
			read_double(p.y);
			read_double(p.x);
			double cc;
			XY(p.y, p.x);
			if(p.x == p0.x && p.y == p0.y)
			{
				j--;
				c--;
				continue;
			}
			else
			{
				cc = max(fabs((p.y - p0.y) / dis), fabs((p.x - p0.x) / dis));
			}
			cmb t1;
			t1.r = i;
			t1.p = p;
			mp[p0].push_back(t1);
			part t2;
			t2.r = i;
			t2.b = p0;
			t2.e = p;
			for(int k = 0; k < cc; k++)
			{
				v[int((p.x * k + p0.x * (cc - k)) / cc) / dis][int((p.y * k + p0.y * (cc - k)) / cc) / dis].push_back(t2);
			}
			v[int(p.x / dis)][int(p.y / dis)].push_back(t2);
			p0 = p;
		}
	}
}
void read_track()
{
	int m;
	scanf("%d", &m);
	printf("%d\n", m);
	for(int i = 0; i != m; i++)
	{
		int top = 0;
		int t;
		scan_d(t);
		while(t != i)
		{
			read_double(track[top].y);
			read_double(track[top].x);
			scan_d(t);
			XY(track[top].y, track[top].x);
			top++;
		}
		vector<vector<int> > r(top), bosi(top);
		vector<vector<double> > b(top), delta(top);
		vector<vector<point> > x(top), head(top), tail(top);
		for(int j = 0; j != top; j++)
		{
			get_near(track[j], head[j], tail[j], x[j], b[j], r[j]);
		}
		delta[0] = b[0];
		bosi[0].resize(r[0].size());
		for(int j = 1; j != top; j++)
		{
			delta[j].resize(r[j].size());
			bosi[j].resize(r[j].size());
			for(int k = 0; k != delta[j].size(); k++)
			{
				for(int l = 0; l != delta[j - 1].size(); l++)
				{
					if(delta[j][k] >= delta[j - 1][l] / beta)
					{
						continue;
					}
					double a;
					if(r[j - 1][l] == r[j][k])
					{
						a = 1 / beta;
					}
					else if(e[r[j - 1][l]].end == e[r[j][k]].start)
					{
						a = 0.86 / beta;
					}
					else
					{
						a = trans(track[j - 1], track[j], tail[j - 1][l], head[j][k], x[j - 1][l], x[j][k]);
					}
					if(delta[j][k] < delta[j - 1][l] * a)
					{
						delta[j][k] = delta[j - 1][l] * a;
						bosi[j][k] = l;
					}
				}
				delta[j][k] *= b[j][k];
			}
		}
		vector<int> cand(top);
		double mm = 0;
		for(int j = 0; j != delta[top - 1].size(); j++)
		{
			if(delta[top - 1][j] > mm)
			{
				cand[top - 1] = j;
				mm = delta[top - 1][j];
			}
		}
		for(int j = top - 1; j; j--)
		{
			cand[j - 1] = bosi[j][cand[j]];
		}
		for(int j = 0; j != top; j++)
		{
			printf("%d ", r[j][cand[j]]);
		}
		putchar('\n');
	}
}
void get_near(point pt, vector<point>& h, vector<point>& t, vector<point>& x, vector<double>& pb, vector<int>& r)
{
	int itx = pt.x / dis, ity = pt.y / dis;
	vector<point> t_h, t_t, t_x;
	vector<double> t_pb;
	vector<int> t_r;
	double mm = 0;
	map<int, int> mr; 
	int xs[9] = {0, 0, -1, 0, 1, -1, -1, 1, 1}, ys[9] = {0, 1, 0, -1, 0, 1, -1, 1, -1};
	for(int i = 0; i != 9; i++)
	{
		int xi = xs[i], yi = ys[i];
		for(int j = 0; j != v[itx + xi][ity + yi].size(); j++)
		{
			int flag;
			part pat = v[itx + xi][ity + yi][j];
			point tpt;
			flag = mr[pat.r] - 1;
			if(flag == -1)
			{
				double tp = dpl(pt, pat.b, pat.e, tpt);
				if(tp)
				{
					mm = max(tp, mm);
					t_h.push_back(pat.b);
					t_t.push_back(pat.e);
					t_x.push_back(tpt);
					t_pb.push_back(tp);
					t_r.push_back(pat.r);
					mr[pat.r] = t_r.size();
				}
			}
			else
			{	
				double tp = dpl(pt, pat.b, pat.e, tpt);
				if(tp > t_pb[flag])
				{
					mm = max(tp, mm);
					t_h[flag] = pat.b;
					t_t[flag] = pat.e;
					t_x[flag] = tpt;
					t_pb[flag] = tp;
				}
			}
		}
	}
	for(int i = 0; i < t_r.size(); i++)
	{
		if(t_pb[i] > mm / 1.8)
		{
			h.push_back(t_h[i]);
			t.push_back(t_t[i]);
			x.push_back(t_x[i]);
			pb.push_back(t_pb[i]);
			r.push_back(t_r[i]);
		}
	}
}
double dpl(point p, point p1, point p2, point& pp)
{
	point d;
	d.x = (p2.x - p1.x) / sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
	d.y = (p2.y - p1.y) / sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
	if((p1.x - p.x) * (p2.x - p1.x) + (p1.y - p.y) * (p2.y - p1.y) >= 0)
	{
		pp.x = p1.x;
		pp.y = p1.y;
		return lacp(sqrt((p.x - p1.x) * (p.x - p1.x) + (p.y - p1.y) * (p.y - p1.y)));
	}
	if((p2.x - p1.x) * (p.x - p2.x) + (p2.y - p1.y) * (p.y - p2.y) >= 0)
	{
		pp.x = p2.x;
		pp.y = p2.y;
		return lacp(sqrt((p.x - p2.x) * (p.x - p2.x) + (p.y - p2.y) * (p.y - p2.y)));
	}
	point dr;
	dr.x = p.x - p1.x;
	dr.y = p.y - p1.y;
	double l = dr.x * d.x + dr.y * d.y;
	pp.x = p1.x + l * d.x;
	pp.y = p1.y + l * d.y;
	return lacp((sqrt((p.x - pp.x) * (p.x - pp.x) + (p.y - pp.y) * (p.y - pp.y))));
}
double lacp(double d)
{
	if(d >= 200)
	{
		return 0;
	}
	else
	{
		return exp(-0.5 * (d * d) / (sigma * sigma));
	}
}
double trans(point z0, point z, point h, point t, point x0, point x)
{
	double l = fabs(route(h, t) + dpp(h, x0) + dpp(t, x) - dpp(z, z0));
	if(l > 200)
	{
		return 1e-32;
	}
	return exp(-l / beta) / beta;
}
double route(point a, point b)
{
	if(a == b)
	{
		return 0;
	}
	for(int i = 0; i != mp[a].size(); i++)
	{
		if(mp[a][i].p == b)
		{
			return dpp(a, b);
		}
	}
	return 10000.0;
}
double dpp(point a, point b)
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
int main()
{
	read_map();
	read_track();
}