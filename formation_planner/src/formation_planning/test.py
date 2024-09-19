# import random
# def select_strategy(previous_str):
#     if (previous_str == 2):
#         return random.choice([0, 1])
#     else:
#         return previous_str



# pay_off_matrix = {
#     (0, 0) : (4, 4),
#     (0, 1) : (1, 3),
#     (1, 0) : (3, 1),
#     (1, 1) : (2, 2)
# }

# # main loop
# total_payoff = 0.0
# T  = random.randint(100, 1000)
# for _ in range(T):
#     previous_str = random.choice([0, 1])
#     me_str = select_strategy(previous_str)
#     payoff = pay_off_matrix[(me_str, previous_str)]
#     total_payoff += payoff[0]
# print("total payoff: ", total_payoff)
import numpy as np

Q = np.array([5, 10, 15, 20, 25])
c = np.array([25, 20, 15, 10, 5])

n = len(Q)
sum_Q = np.sum(Q)
sum_c = np.sum(c)
sum_Qc = np.sum(Q * c)
sum_Q2 = np.sum(Q**2)

a = (n * sum_Qc - sum_Q * sum_c) / (n * sum_Q2 - sum_Q**2)
b = (sum_c - a * sum_Q) / n

print(a, b)
