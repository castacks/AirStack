"""Analytic ambiguity and scaled-Jacobian controls, separate from inference."""
import csv
import math


def singular(matrix):
    rows, columns = len(matrix), len(matrix[0])
    size = min(rows, columns)
    if size == 1:
        return [math.sqrt(sum(value*value for row in matrix for value in row))] + [0] * (columns-1)
    vectors = matrix if rows == 2 else list(zip(*matrix))
    a, b = vectors
    trace = sum(x*x + y*y for x, y in zip(a, b, strict=True))
    determinant = math.fsum((a[i]*b[j]-a[j]*b[i])**2
                            for i in range(len(a)) for j in range(i+1, len(a)))
    maximum = (trace + math.sqrt(max(0, trace*trace - 4*determinant))) / 2
    minimum = determinant / maximum if maximum else 0
    return [math.sqrt(maximum), math.sqrt(minimum)] + [0] * (columns-2)


def cases():
    def rgb(b):
        return .015 + (.19-.015)*math.exp(-10*(.003+.135*b))

    def lw(b):
        return 41 + 15*math.exp(-10*(.002+.024*b))

    b = .5
    rgb_j = -1.35 * (rgb(b) - .015)
    lwir_j = -.24 * (lw(b) - 41)
    for name, fn, jacobian, sigma in (("rgb_known", lambda q: [rgb(q[0])], [[rgb_j]], [.007]),
            ("lwir_known", lambda q: [lw(q[0])], [[lwir_j]], [.55]),
            ("joint_known", lambda q: [rgb(q[0]), lw(q[0])], [[rgb_j], [lwir_j]], [.007, .55])):
        yield name, fn, [b], jacobian, sigma, ""
    a, b = .7, .24
    rgb_j, lwir_j = -a*math.exp(-a), -b*math.exp(-b)
    yield ("unknown_range_one_band", lambda q: [math.exp(-a*math.exp(q[0]+q[1]))],
           [0, 0], [[rgb_j, rgb_j]], [.01], "1,-1")
    yield ("unknown_range_two_bands", lambda q: [math.exp(-a*math.exp(q[0]+q[2])),
                                                 math.exp(-b*math.exp(q[1]+q[2]))],
           [0, 0, 0], [[rgb_j,0,rgb_j],[0,lwir_j,lwir_j]], [.01,.01], "-1,-1,1")
    for name, constant in (("lwir_crossover", 46), ("target_occluded", 25), ("rail_plateau", 1)):
        yield name, lambda _: [constant], [.5], [[0]], [.55], "1"


def audit(output):
    largest = 0
    ranks = {}
    information = {}
    with output.open("w", newline="") as file:
        out = csv.writer(file, lineterminator="\n")
        out.writerow(("case", "rank", "dimension", "sigma_values", "information_eigenvalues",
                      "full_condition", "null_direction", "max_fd_error"))
        for name, fn, point, jac, noise, null in cases():
            numerical = []
            delta = 1e-5
            for i in range(len(point)):
                lower, upper = list(point), list(point)
                lower[i] -= delta
                upper[i] += delta
                numerical.append([(a-b)/(2*delta) for a,b in zip(fn(upper),fn(lower),strict=True)])
            error = max(abs(a-b) for calculated, expected in zip(zip(*numerical), jac, strict=True)
                        for a,b in zip(calculated,expected,strict=True))
            values = singular(jac)
            whitened = [[value/s for value in row] for row,s in zip(jac,noise,strict=True)]
            eigen = [value**2 for value in singular(whitened)]
            rank = sum(value > values[0]*1e-8 for value in values)
            cond = values[0]/values[-1] if rank == len(point) else ""
            out.writerow((name,rank,len(point),values,eigen,cond,null,error))
            ranks[name], information[name] = rank, eigen[0]
            largest = max(largest,error)
    return ranks, information, largest


def oracle(output, planck_band):
    """Privileged exact observations and explicit equal-output counterexamples."""
    extinction, length = .08, 12
    transmission = math.exp(-extinction*length)
    recovery = -math.log(transmission)/length
    same_depth = math.exp(-(.5*extinction)*(2*length))
    source = .02
    first = source + (.2-source)*transmission
    other_t = math.exp(-.35)
    other_surface = source + (first-source)/other_t
    same_source = source + (other_surface-source)*other_t
    reflected, blackbody = planck_band(285), planck_band(310)
    band = .8*blackbody + .2*reflected
    emissivity = (band-reflected)/(planck_band(330)-reflected)
    same_thermal = emissivity*planck_band(330)+(1-emissivity)*reflected
    with output.open("w", newline="") as file:
        out = csv.writer(file, lineterminator="\n")
        out.writerow(("oracle_control", "observation", "alternate_observation", "constraint"))
        out.writerows([
            ("known_range", recovery, extinction, "exact transmission and independent L"),
            ("unknown_range", transmission, same_depth, "(0.08,12) and (0.04,24)"),
            ("unknown_source", first, same_source, f"source=0.02,target2={other_surface}"),
            ("emissivity_temperature", band, same_thermal, f"(310,0.8) vs (330,{emissivity})"),
        ])
    return abs(recovery-extinction), max(abs(transmission-same_depth),abs(first-same_source),
                                        abs(band-same_thermal)), emissivity
