import numpy as np
import random
from scipy import optimize
import matplotlib.pyplot as plt


def load_data(start=399):
    data = np.loadtxt("data\multi_control_data.txt", delimiter=",")
    uc = data[:, 0]
    uh = data[:, 1]
    up = data[:, 2]
    Vc = data[:, 3]
    Tco = data[:, 4]
    Thi = data[:, 5]

    # Get the data from the designated starting point
    uc = uc[start:]
    Vc = Vc[start:]
    uh = uh[start:]
    Tco = Tco[start:]
    up = up[start:]
    Thi = Thi[start:]
    return uc, uh, up, Vc, Tco, Thi


def plot_data(**kwargs):
    length_u = len(next(iter(kwargs.values())))
    no_input = len(kwargs)

    # Style
    style = ["r-", "g--", "b-", "y--", "m-", "c--"]
    if no_input > len(style):
        for i in range(no_input // len(style)):
            style.extend(style[:])

    # Time points
    T = length_u - 1
    t = np.linspace(0, T, T + 1)
    time = np.arange(T)

    # Plot inputs
    fig, ax = plt.subplots()
    for i, (var_name, var_value) in enumerate(kwargs.items()):
        ax.plot(t, var_value, style[i], label=var_name)
    ax.set_xlabel("Time")
    ax.legend()
    plt.show()


def mini(u, y, r, initials):
    # Minimizer
    def u_error(x):
        Kc, Ti = x
        sum_error = 0
        for i, Uk in enumerate(u):
            error = abs(
                u[i - 1] + Kc * ((1 + Ts / Ti) * (r - y[i]) - (r - y[i - 1])) - Uk
            )
            sum_error += error
        return sum_error / len(u)

    methods = [
        "Nelder-Mead",
        "Powell",
        "CG",
        "BFGS",
        "L-BFGS-b",
        "TNC",
        "COBYLA",
        "SLSQP",
        "trust-constr",
    ]

    results = []
    for initial in initials:
        for method in methods:
            try:
                a = optimize.minimize(u_error, initial, method=method)
                result = (a.fun, a.x, method)  # make an object for saving results
                results.append(result)
            except Exception as e:
                print(f"Raise error: {e}")
    return results


def save_list_to_file(sorted_outer_list: list[list]):
    with open("data\solution_list.txt", "w") as file:
        for item in sorted_outer_list:
            file.write(str(item) + "\n")


def initialize_controller_values(uc, uh, up, Vc, Tco, Thi, r_Vc, r_Tco, r_Thi, seed=10):
    random.seed(seed)
    random_initial_value = [
        [random.uniform(-1, 5), random.uniform(1, 100)] for _ in range(1)
    ]
    controllers = [
        (uc, Vc, r_Vc, [[0.5, 0.5]]),
        (uh, Tco, r_Tco, random_initial_value),
        (up, Thi, r_Thi, random_initial_value),
    ]
    return controllers


def find_controller_parameters(mini, controller_init_values):
    results = [mini(*controller) for controller in controller_init_values]
    sorted_results = [sorted(result, key=lambda x: x[0]) for result in results]
    return sorted_results


if __name__ == "__main__":
    uc, uh, up, Vc, Tco, Thi = load_data()

    # Setpoints
    r_Vc = 200
    r_Tco = 40
    r_Thi = 50
    Ts = 1

    plot_data(uc=uc)
    controller_init_values = initialize_controller_values(
        uc, uh, up, Vc, Tco, Thi, r_Vc, r_Tco, r_Thi
    )
    sorted_results = find_controller_parameters(mini, controller_init_values)

    for i, inner_list in enumerate(sorted_results):
        print(i, "sorted list:", inner_list, "\n")

    # Print the answer
    uc_best = sorted_results[0][0]
    uh_best = sorted_results[1][0]
    up_best = sorted_results[2][0]
    uc_fit, (uc_Kc, uc_Ti), _ = uc_best
    uh_fit, (uh_Kc, uh_Ti), _ = uh_best
    up_fit, (up_Kc, up_Ti), _ = up_best
    print(
        f"uc controller's fitness is {uc_fit:.2e}, "
        f"using gain(Kc): {uc_Kc:.2f}, "
        f"and integration time(Ti): {uc_Ti:.2f}"
    )
    print(
        f"uh controller's fitness is {uh_fit:.2e}, "
        f"using gain(Kc): {uh_Kc:.2f}, "
        f"and integration time(Ti): {uh_Ti:.2f}"
    )
    print(
        f"up controller's fitness is {up_fit:.2e}, "
        f"using gain(Kc): {up_Kc:.2f}, "
        f"and integration time(Ti): {up_Ti:.2f}"
    )

    save_list_to_file(sorted_results)
