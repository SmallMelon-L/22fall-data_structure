#include <bits/stdc++.h>
#define PI 3.14
#define dis 60
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
double trans(point, point, point, point, point, point, point, point);
double route(point, point);
double dpp(point, point);
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
		cin >> e[i].level;
		scanf("%d%d%lf%lf", &e[i].lv, &c, &p0.y, &p0.x);
		XY(p0.y, p0.x);
		for(int j = 1; j != c; j++)
		{
			scanf("%lf%lf", &p.y, &p.x);
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
		scanf("%d", &t);
		while(t != i)
		{
			scanf("%lf%lf%d", &track[top].y, &track[top].x, &t);
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
					a = trans(track[j - 1], track[j], head[j - 1][l], tail[j - 1][l], head[j][k], tail[j][k], x[j - 1][l], x[j][k]);
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
	int xs[9] = {0, 0, -1, 0, 1, -1, -1, 1, 1}, ys[9] = {0, 1, 0, -1, 0, 1, -1, 1, -1};
	for(int i = 0; i != 9; i++)
	{
		int xi = xs[i], yi = ys[i];
		for(int j = 0; j != v[itx + xi][ity + yi].size(); j++)
		{
			int flag = -1;
			part pat = v[itx + xi][ity + yi][j];
			point tpt;
			for(int k = 0; k != t_r.size(); k++)
			{
				if(t_r[k] == pat.r && t_h[k] == pat.b)
				{
					flag = k;
					break;
				}
			}
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
				}
			}
		}
	}
	for(int i = 0; i < t_r.size(); i++)
	{
		if(t_pb[i] > mm / 2.9)
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
double trans(point z0, point z, point h0, point t0, point h, point t, point x0, point x)
{
	double l;
	if(h0 == h && t0 == t)
	{
		l = fabs(dpp(x, x0) - dpp(z, z0));
	}
	else
	{
		l = fabs(route(t0, h) + dpp(t0, x0) + dpp(h, x) - dpp(z, z0));
	}
	if(l > 100)
	{
		return 1e-30;
	}
	return exp(-l / beta) / beta;
}
double route(point a, point b)
{
	if(a == b)
	{
		return 0;
	}
	double mm = 10000.0;
	vector<point> p;
	vector<double> d;
	map<point, int> al;
	al[a] = 1;
	p.push_back(a);
	d.push_back(0);
	int k = 0, s = 0, e = 1;
	while(k++ != 6)
	{
		for(int i = s; i != e; i++)
		{
			for(int j = 0; j != mp[p[i]].size(); j++)
			{
				double pp = d[i] + dpp(mp[p[i]][j].p, p[i]);
				if(dpp(mp[p[i]][j].p, b) > dpp(a, b) * 1.5)
				{
					continue;
				}
				if(mp[p[i]][j].p == b)
				{
					mm = min(mm, pp);
				}
				if(al[mp[p[i]][j].p])
				{
					continue;
				}
				else
				{
					al[mp[p[i]][j].p] = 1;
				}
				p.push_back(mp[p[i]][j].p);
				d.push_back(pp);
			}
		}
		if(mm != 10000.0)
		{
			return mm;
		}
		s = e;
		e = p.size();
	}
	return mm;
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