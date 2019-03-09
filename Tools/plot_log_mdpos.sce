clear();
clf();

f = scf(0);
clf(f, "clear");
f.figure_size = [1500, 1000];

cd "/home/yusaku/md201x/logs";

filename = "20190304_00-21-39";

raw = csvRead("./" + filename + ".csv");

raw_bounds = 2:5000;

t = raw(raw_bounds, 1);
t = t - t(2)

t_bounds = [0; 9000];
//t_bounds = [2950; 3050];

subplot(4, 1, 1);

pos_cur = raw(raw_bounds, 2);
pos_tgt = raw(raw_bounds, 3);

plot(t, pos_tgt, "b-");
plot(t, pos_cur, "r-");

a = gca();
a.tight_limits = "on";
a.data_bounds(:, 1) = t_bounds;
//a.data_bounds(:, 2) = [240; 260];
legend(["$\theta_{target}$", "$\theta_{current}$"], 3);
xgrid();

subplot(4, 1, 2);

vel_cur = raw(raw_bounds, 4);
vel_tgt = raw(raw_bounds, 5);

plot(t, vel_tgt, "b-");
plot(t, vel_cur, "r-");

a = gca();
a.tight_limits = "on";
a.data_bounds(:, 1) = t_bounds;
legend(["$\omega_{target}$", "$\omega_{current}$"], 3);
xgrid();

subplot(4, 1, 3);

u_p = raw(raw_bounds, 8);
u_i = raw(raw_bounds, 9);

plot(t, u_p, "b-");
plot(t, u_i, "r-");

a = gca();
a.tight_limits = "on";
a.data_bounds(:, 1) = t_bounds;
a.data_bounds(:, 2) = [-5; 5];
legend(["$\Delta u_p$", "$\Delta u_i$"], 3);
xgrid();

subplot(4, 1, 4);

tgt_trq = raw(raw_bounds, 10);
tgt_vol = raw(raw_bounds, 11);

plot(t, tgt_trq, "b-");
plot(t, tgt_vol, "r-");

a = gca();
a.tight_limits = "on";
a.data_bounds(:, 1) = t_bounds;
legend(["$\tau_{target}$", "$V_{target}$"], 3);
xgrid();

xs2pdf(0, "./figs/md201x_" + filename + ".pdf");
xs2png(0, "./figs/md201x_" + filename + ".png");



