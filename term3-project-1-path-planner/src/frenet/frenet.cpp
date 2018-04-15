#include "frenet.h"

Frenet::Frenet(const vector<double> &maps_s,
	    const vector<double> &maps_x,
        const vector<double> &maps_y) {
	this->map_s = maps_s;
	this->map_x = maps_x;
	this->map_y = maps_y;
}

double Frenet::distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int Frenet::closest_waypoint(double x, double y) {

	double closest_len = 100000;
	int closest_wp = 0;

	for (int i = 0; i < map_x.size(); i++) {
		double dist = distance(x, y, map_x[i], map_y[i]);
		if (dist < closest_len) {
			closest_len = dist;
			closest_wp = i;
		}
	}

	return closest_wp;
}

int Frenet::next_waypoint(double x, double y, double theta) {

	int closest_wp = closest_waypoint(x, y);

	double mapx = map_x[closest_wp];
	double mapy = map_y[closest_wp];

	double heading = atan2((mapy - y), (mapx - x));

	double angle = fabs(theta - heading);
    angle = min(2 * M_PI - angle, angle);

    if (angle > M_PI / 4) {
        closest_wp++;

        if (closest_wp == map_x.size()) {
            closest_wp = 0;
        }
    }
    return closest_wp;
}

void Frenet::from_cartesian(double x, double y, double theta) {
    int next_wp = next_waypoint(x, y, theta);
	int prev_wp = next_wp-1;
	if(next_wp == 0) {
		prev_wp  = map_x.size()-1;
	}

	double n_x = map_x[next_wp] - map_x[prev_wp];
	double n_y = map_y[next_wp] - map_y[prev_wp];
	double x_x = x - map_x[prev_wp];
	double x_y = y - map_y[prev_wp];

	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	double center_x = 1000 - map_x[prev_wp];
	double center_y = 2000 - map_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if(centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++) {
		frenet_s += distance(map_x[i], map_y[i], map_x[i+1], map_y[i+1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	this->s_ = frenet_s;
	this->d_ = frenet_d;
	this->x_ = x;
	this->y_ = y;
}

void Frenet::to_cartesian(double s, double d) {
	int prev_wp = -1;

	while(s > map_s[prev_wp + 1] && (prev_wp < (int)(map_s.size() - 1) )) {
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % map_x.size();

	double heading = atan2((map_y[wp2] - map_y[prev_wp]), (map_x[wp2] - map_x[prev_wp]));
	double seg_s = (s - map_s[prev_wp]);

	double seg_x = map_x[prev_wp] + seg_s * cos(heading);
	double seg_y = map_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - M_PI / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	this->x_ = x;
	this->y_ = y;
	this->s_ = s;
	this->d_ = d;
}