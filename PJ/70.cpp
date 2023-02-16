#include <bits/stdc++.h>
#define PI 3.1415926535897932384626
#define dis 250
#define sigma 30.0
using namespace std;
struct point
{
	double x, y;
};
struct part
{
	int r;
	point b, e;
};
struct cmb
{
	int r;
	point p;
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
vector<part> v[500][300];
point track[2100];
void read_map();
void read_track();
void XY(double&, double&);
int get_near(point, point);
double dpl(point, point, point, point&);
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
void XY(double& y, double& x)
{
	y -= 30.6;
	x -= 121.003;
	y *= 111000;
	x *= 96126;
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
void read_map()
{
	int n;
	scanf("%d", &n);
	for(int i = 0; i != n; i++)
	{
		int c;
		point p0, p;
		scan_d(c);
		scan_d(c);
		scan_d(c);
		getchar();
		while(getchar() != ' ');
		scan_d(c);
		scan_d(c);
		read_double(p0.y);
		read_double(p0.x);
		XY(p0.y, p0.x);
		for(int j = 1; j != c; j++)
		{
			read_double(p.y);
			read_double(p.x);
			XY(p.y, p.x);
			double cc;
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
			part t;
			t.r = i;
			t.b = p0;
			t.e = p;
			for(int k = 1; k < cc; k++)
			{
				v[int((p.x * k + p0.x * (cc - k)) / cc) / dis][int((p.y * k + p0.y * (cc - k)) / cc) / dis].push_back(t);
			}
			v[int(p.x / dis)][int(p.y / dis)].push_back(t);
			p0 = p;
		}
	}
}
void read_track()
{
	int m;
	scan_d(m);
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
		point v;
		for(int j = 0; j != top - 1; j++)
		{
			v.x = track[j + 1].x - track[j].x;
			v.y = track[j + 1].y - track[j].y;
			int out = get_near(track[j], v);
			printf("%d ", out);
		}
		v.x = v.y = 0;
		printf("%d\n", get_near(track[top - 1], v));
	}
}
int get_near(point pt, point vp)
{
	int itx = pt.x / dis, ity = pt.y / dis;
	double mm = 200;
	int r;
	part pat;
	int s = v[itx][ity].size();
	for(int j = 0; j != s; j++)
	{
		pat = v[itx][ity][j];
		if(vp.x * (pat.e.x - pat.b.x) + vp.y * (pat.e.y - pat.b.y) < 0)
		{
			continue;
		}
		point tpt;
		double d = dpl(pt, pat.b, pat.e, tpt);
		if(d < mm)
		{
			mm = d;
			r = pat.r;
		}
	}
	return r;
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
		return sqrt((p.x - p1.x) * (p.x - p1.x) + (p.y - p1.y) * (p.y - p1.y));
	}
	if((p2.x - p1.x) * (p.x - p2.x) + (p2.y - p1.y) * (p.y - p2.y) >= 0)
	{
		pp.x = p2.x;
		pp.y = p2.y;
		return sqrt((p.x - p2.x) * (p.x - p2.x) + (p.y - p2.y) * (p.y - p2.y));
	}
	point dr;
	dr.x = p.x - p1.x;
	dr.y = p.y - p1.y;
	double l = dr.x * d.x + dr.y * d.y;
	pp.x = p1.x + l * d.x;
	pp.y = p1.y + l * d.y;
	return sqrt((p.x - pp.x) * (p.x - pp.x) + (p.y - pp.y) * (p.y - pp.y));
}
int main()
{
	read_map();
	read_track();
}