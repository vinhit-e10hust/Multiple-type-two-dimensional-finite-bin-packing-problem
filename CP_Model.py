from ortools.sat.python import cp_model
import sys
import time
def Input():
    data= {}
    f = open("0010.txt",'r')
    [N,K] = [int(x) for x in f.readline().split()]
    item_sizes = []
    truck_sizes = []
    truck_costs = []

    for i in range(N):
        item_sizes.append(list(map(int, f.readline().split()))) 

    for i in range(K):
        wi,li,ci = list(map(int,f.readline().split()))
        truck_sizes.append([wi,li])
        truck_costs.append(ci)

    return N, K, item_sizes, truck_sizes, truck_costs

N, K, item_sizes, truck_sizes, truck_costs = Input()

# Tạo model
model = cp_model.CpModel()

def main_solver():
    start_time = time.time()
    # Khởi tạo biến
    X = [[model.NewBoolVar(f'x_{i}_{j}') for j in range(K)] for i in range(N)]
    R = [model.NewBoolVar(f'r_{i}') for i in range(N)]
    Z = [model.NewBoolVar(f'z_{i}') for i in range(K)]

    max_width = max(truck[0] for truck in truck_sizes)
    max_length = max(truck[1] for truck in truck_sizes)

    # Tọa độ trong xe(left/right/top/bottom)
    l, r, t, b = [], [], [], []
    for i in range(N):
        l.append(model.NewIntVar(0, max_width, f'l_{i}'))
        r.append(model.NewIntVar(0, max_width, f'r_{i}'))
        t.append(model.NewIntVar(0, max_length, f't_{i}'))
        b.append(model.NewIntVar(0, max_length, f'b_{i}'))

        # Constraints quay hoặc cố định
        model.Add(r[i] == l[i] + item_sizes[i][0]).OnlyEnforceIf(R[i].Not())
        model.Add(r[i] == l[i] + item_sizes[i][1]).OnlyEnforceIf(R[i])
        model.Add(t[i] == b[i] + item_sizes[i][1]).OnlyEnforceIf(R[i].Not())
        model.Add(t[i] == b[i] + item_sizes[i][0]).OnlyEnforceIf(R[i])

    # Constraints
    # Mỗi vật phẩm chỉ đặt trong duy nhất 1 xe
    for i in range(N):
        model.Add(sum(X[i][j] for j in range(K)) == 1)

    # Đảm bảo các vật phẩm vừa với kích thước xe tải được chọn
    for i in range(N):
        for j in range(K):
            model.Add(r[i] <= truck_sizes[j][0]).OnlyEnforceIf(X[i][j])
            model.Add(t[i] <= truck_sizes[j][1]).OnlyEnforceIf(X[i][j])

    # Đảm bảo các vật phẩm không bị đè lên nhau
    for i in range(N):
        for k in range(i + 1, N):
            # Khơi tạo điều kiện không bị đè lên nhau
            a1 = model.NewBoolVar(f'a1_{i}_{k}')
            a2 = model.NewBoolVar(f'a2_{i}_{k}')
            a3 = model.NewBoolVar(f'a3_{i}_{k}')
            a4 = model.NewBoolVar(f'a4_{i}_{k}')
            # Xếp vật phẩm mới ko được chồng lên vật cũ
            model.Add(r[i] <= l[k]).OnlyEnforceIf(a1)
            model.Add(t[i] <= b[k]).OnlyEnforceIf(a2)
            model.Add(r[k] <= l[i]).OnlyEnforceIf(a3)
            model.Add(t[k] <= b[i]).OnlyEnforceIf(a4)

            # Chỉ áp dụng điều kiện không bị đè lên nhau khi 2 gói hàng cùng thuộc 1 xe
            for j in range(K):
                model.AddBoolOr([a1, a2, a3, a4]).OnlyEnforceIf(X[i][j]).OnlyEnforceIf(X[k][j])

    # Tìm xe được sử dụng
    for j in range(K):
        truck_used = model.NewBoolVar(f'truck_used_{j}')

        model.AddBoolOr([X[i][j] for i in range(N)]).OnlyEnforceIf(truck_used)
        model.AddBoolAnd([X[i][j].Not() for i in range(N)]).OnlyEnforceIf(truck_used.Not())

        model.Add(Z[j] == 1).OnlyEnforceIf(truck_used)
        model.Add(Z[j] == 0).OnlyEnforceIf(truck_used.Not())

    # Khai báo hàm mục tiêu
    cost = sum(Z[j] * truck_costs[j] for j in range(K))
    model.Minimize(cost)

    # Cú pháp giải modeling
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 300
    status = solver.Solve(model)


    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        res = []
        for i in range(N):
            try:
                pos_truck = next(j for j in range(K) if solver.Value(X[i][j]) == 1)
                x_toado = solver.Value(l[i])
                y_toado = solver.Value(b[i])

                if solver.Value(R[i])==1:
                  rotation = 1
                else:
                  rotation = 0

                res.append([i + 1, pos_truck + 1, x_toado, y_toado, rotation])
            except StopIteration:
                res.append([i + 1, -1, -1, -1, -1])
        for line in res:
            print(" ".join(map(str, line)))
        print(int(solver.ObjectiveValue()))

        end_time = time.time()
        diff = end_time - start_time
        print(f'{diff:.10f} sec')
    else:
        print("No solution found.")

if __name__ == "__main__":
    main_solver()
