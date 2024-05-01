import ast
import collections
from ortools.sat.python import cp_model


# Получение данных
def get_data() -> list:
    # Данные
    jobs_data = []

    # Красивый ввод данных
    while True:
        job_input = input(
            "Введите данные по форме: [(номер станка, время выполнения),(номер станка, время выполнения),...], или введите 'end' для окончания ввода: ")

        if job_input.lower() == 'end':
            break

        try:
            job = ast.literal_eval(job_input)
            if not isinstance(job, list):
                raise ValueError
            for task in job:
                if not isinstance(task, tuple) or len(task) != 2 or not all(isinstance(val, int) for val in task):
                    raise ValueError
            jobs_data.append(job)
        except (ValueError, SyntaxError):
            print(
                "Неверный ввод. Пожалуйста введите данные по форме: [(номер станка, время выполнения),(номер станка, время выполнения),...]. Например: [(0, 3), (1, 2), (2, 2)]")

    print("\n", "Данные введены.")
    # print("jobs_data =", jobs_data)
    return jobs_data


# Создаем модель
def create_model(jobs_data: list, horizon: int, all_machines: range) -> tuple:
    model = cp_model.CpModel()

    # Тут именуем массивы
    task_type = collections.namedtuple("task_type", "start end interval")
    assigned_task_type = collections.namedtuple("assigned_task_type", "start job index duration")

    # Создает интервалы выполнения заданий и добавляет их в соответствующие списки машин
    all_tasks = {}
    machine_to_intervals = collections.defaultdict(list)

    for job_id, job in enumerate(jobs_data):
        for task_id, task in enumerate(job):
            machine, duration = task
            suffix = f"_{job_id}_{task_id}"
            start_var = model.NewIntVar(0, horizon, f"start{suffix}")
            end_var = model.NewIntVar(0, horizon, f"end{suffix}")
            interval_var = model.NewIntervalVar(start_var, duration, end_var, f"interval{suffix}")
            all_tasks[job_id, task_id] = task_type(start=start_var, end=end_var, interval=interval_var)
            machine_to_intervals[machine].append(interval_var)

    # Создаваем и добавляйте дизъюнктивные ограничения
    for machine in all_machines:
        model.AddNoOverlap(machine_to_intervals[machine])

    # Приоритеты внутри задания
    for job_id, job in enumerate(jobs_data):
        for task_id in range(len(job) - 1):
            model.Add(all_tasks[job_id, task_id + 1].start >= all_tasks[job_id, task_id].end)

    # Makespan объекты
    obj_var = model.NewIntVar(0, horizon, "makespan")
    model.AddMaxEquality(
        obj_var,
        [all_tasks[job_id, len(job) - 1].end for job_id, job in enumerate(jobs_data)]
    )
    model.Minimize(obj_var)

    return model, all_tasks, machine_to_intervals, assigned_task_type


def solve_model(model: cp_model.CpModel, all_tasks: dict, jobs_data: list, assigned_task_type: collections.namedtuple):
    # Создаем solver и проходимся им по модели
    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    # Создаем список станков
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        assigned_jobs = collections.defaultdict(list)
        for job_id, job in enumerate(jobs_data):
            for task_id, task in enumerate(job):
                machine = task[0]
                assigned_jobs[machine].append(
                    assigned_task_type(
                        start=solver.Value(all_tasks[job_id, task_id].start),
                        job=job_id,
                        index=task_id,
                        duration=task[1],
                    )
                )
        return solver, assigned_jobs
    else:
        return None, None


def print_solution(solver: cp_model.CpSolver, assigned_jobs: dict, all_machines: range) -> None:
    if solver:

        # Создание строки для каждого станка
        output = ""
        for machine in all_machines:
            # Сортировка по началу работы
            assigned_jobs[machine].sort()
            sol_line_tasks = f"Станок {machine}: "
            sol_line = "           "

            for assigned_task in assigned_jobs[machine]:
                name = f"job_{assigned_task.job}_task_{assigned_task.index}"
                # Добавляем промежутки
                sol_line_tasks += f"{name:15}"

                start = assigned_task.start
                duration = assigned_task.duration
                sol_tmp = f"[{start},{start + duration}]"
                # Обнавляем промежутки
                sol_line += f"{sol_tmp:15}"

            sol_line += "\n"
            sol_line_tasks += "\n"
            output += sol_line_tasks
            output += sol_line

        # Вывод решения
        print(f"Оптимальное время: {solver.ObjectiveValue()}")
        print(output)
    else:
        print("Решение не найдено.")


def main() -> None:
    jobs_data = get_data()

    machines_count = 1 + max(task[0] for job in jobs_data for task in job)
    all_machines = range(machines_count)

    # Динамически вычисляем горизонт как сумму всех длин
    horizon = sum(task[1] for job in jobs_data for task in job)

    model, all_tasks, machine_to_intervals, assigned_task_type = create_model(jobs_data, horizon,
                                                                              all_machines)  # Тут мы ее создали

    solver, assigned_jobs = solve_model(model, all_tasks, jobs_data, assigned_task_type)  # Взоимодействуем с моделью

    print_solution(solver, assigned_jobs, all_machines)  # Печатаем решение


if __name__ == "__main__":
    main()
