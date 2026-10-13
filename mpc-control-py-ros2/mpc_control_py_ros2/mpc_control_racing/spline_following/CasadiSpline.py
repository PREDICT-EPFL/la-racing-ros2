import casadi
import casadi as ca
import numpy as np
import yaml

class CasadiSpline:
    class SplineType:
        Pos = 0
        Generic = 1

    class SplineInfo:
        def __init__(self):
            self.pieces = 10
            self.order = 4
            self.dim = 2
            self.spline_length = 10.710722313718323
    spline_info = SplineInfo()
    spline_func = None

    def __init__(self):
        self.set_default_data()
        self.init(CasadiSpline.SplineType.Pos)
        return

    def load_from_yaml(self, yaml_filepath, coeff_key="pos", spline_type=SplineType.Generic):

        # Try to open file
        try:
            with open(yaml_filepath, 'r') as f:
                yamlFile = yaml.safe_load(f)
            print(f"Loaded spline from {yaml_filepath}")
        except Exception as ex:
            print(f"YAML file {yaml_filepath} could not be opened:\n{ex}")
            return

        # Try to read file
        try:
            print(f"Loading spline data: {coeff_key}... ", end="")
            self.break_data = np.array(yamlFile["spline_data"]["breaks"], dtype=float)
            coefs = np.array(yamlFile["spline_data"][coeff_key], dtype=float)
            self.coef_data = ca.DM(coefs)

            self.spline_info.spline_length = yamlFile["spline_info"]["spline_length"]
            self.spline_info.pieces = int(yamlFile["spline_info"]["pieces"])
            self.spline_info.order = int(yamlFile["spline_info"]["order"])
        except Exception as ex:
            print("Loading legacy spline file... ", end="")
            self.break_data = np.array(yamlFile["track"]["breaks_data"], dtype=float)
            coef_data = ca.DM(yamlFile["track"]["coeff_data"], dtype=float)

            self.spline_info.pieces = len(self.break_data) - 1
            self.spline_info.order = 4
            self.coef_data = ca.reshape(coef_data,
                                        self.spline_info.order, self.spline_info.dim * self.spline_info.pieces).T
        print("OK")

        self.init(spline_type)
        return

    # "Protected" members
    break_data = None
    coef_data = None
    curv_coef_data = None

    def set_default_data(self):
        self.spline_info.pieces = 14
        self.spline_info.order = 4

        self.break_data = ca.DM([0,
                                 0.765170641456049,
                                 1.53034128291210,
                                 2.29551192436815,
                                 3.06068256582420,
                                 3.82585320728025,
                                 4.59102384873630,
                                 5.35619449019235,
                                 6.12136513164839,
                                 6.88653577310444,
                                 7.65170641456049,
                                 8.41687705601654,
                                 9.18204769747259,
                                 9.94721833892864,
                                 10.7123889803847
                                 ])

        self.coef_data = ca.DM([0.00522015132294312, -0.00673753476715322, 0.984346331848934, -0.00246129470095528,
                                -0.0197687659904417, -0.0818434306068664, 0.0848776239050533, -0.00925513732634816,
                                0.0164368953173074, 0.00524538484166887, 0.983204582533302, 0.749125497427565,
                                0.221225809843044, -0.127222868667969, -0.0750937704179146, -0.00108380682048688,
                                -0.405814424856174, 0.0429764740421389, 1.02010253322763, 1.51187955471725,
                                0.0118051512615771, 0.380603615804740, 0.118785738401342, -0.0339221267740268,
                                0.170005073446387, -0.888575377295810, 0.373075078010487, 2.13579063631256,
                                -0.469267127337286, 0.407702481294660, 0.721974420382605, 0.285096033337023,
                                0.221861247325611, -0.498326703996547, -0.688141677168717, 1.97716991744362,
                                0.285451688742776, -0.669505805222065, 0.521650203077747, 0.866003640422350,
                                0.219882393024045, 0.0109584347945847, -1.06106156833931, 1.25825387023079,
                                0.289084908902802, -0.0142480498819975, -0.00153817283027566, 1.00105014923253,
                                -0.222344674726119, 0.515701089939884, -0.658077161969297, 0.551283325388401,
                                -0.286332264778738, 0.649349805659265, 0.484423045027679, 1.12104025836893,
                                -0.242179702412380, 0.00530623778632099, -0.259417650809735, 0.250068127646740,
                                -0.225827586260867, -0.00792932247166700, 0.975219167591381, 1.74361590914406,
                                0.342183902561027, -0.550620156941227, -0.676675852124406, -0.0538196416382319,
                                -0.0363216300107808, -0.526319239584763, 0.566427852665689, 2.38401244489134,
                                0.156371166646934, 0.234867071714440, -0.918280842889113, -0.740675304078852,
                                0.193020761735331, -0.609695974386998, -0.302817637312917, 2.49300183037417,
                                -0.354047828683640, 0.593818949379835, -0.284194628562746, -1.23575165777978,
                                0.0711834128266431, -0.166614514172922, -0.896827631813369, 1.99079882019298,
                                0.361497077227009, -0.218902063160113, 0.00268076575870438, -1.26414858517151,
                                0.0867386609438621, -0.00321214121214236, -1.02677400265069, 1.23891210195212,
                                -0.203679788308727, 0.610918788238720, 0.302640454748602, -1.22831193188793,
                                0.164294709545497, 0.195897489288219, -0.879336831264142, 0.490232756693328,
                                -0.0653914878842232, 0.143369405423256, 0.879799635735663, -0.730303765689508,
                                -0.285287729423421, 0.573037954160510, -0.290970004762186, 0.00568878983071339])
        self.coef_data = casadi.reshape(self.coef_data,
                                        self.spline_info.order, self.spline_info.dim * self.spline_info.pieces).T

        # print("Default.\n"
        #       "breaks: ", self.break_data.shape, "\n"
        #       "coefs: ", self.coef_data.shape, "\n")
        return
    def init(self, spline_type=SplineType.Generic):
        print("Init. "
              f"Spline Info: dim: {self.spline_info.dim}, order: {self.spline_info.order}, "
              f"pieces: {self.spline_info.pieces}, spline_length: {self.spline_info.spline_length}\n"
              f"breaks: {self.break_data.shape}\n"
              f"coefs: {self.coef_data.shape}")

        breaks = ca.MX(self.break_data)
        coeff = ca.MX(self.coef_data)

        # Duplicate path to cover positive transition across periodicity
        breaks = ca.vertcat(breaks, breaks[-1] + breaks[1:])
        coeff = ca.vertcat(coeff, coeff)

        t = ca.MX.sym("t")
        t_scaled = self.break_data[-1] * t
        id_low = ca.low(breaks, t_scaled)
        tau = t_scaled - breaks[id_low]

        if spline_type == CasadiSpline.SplineType.Pos:
            coefs_x = coeff[2 * id_low, :]
            coefs_y = coeff[2 * id_low + 1, :]
            pos = ca.vertcat(self.poly_eval_horner(coefs_x, tau), self.poly_eval_horner(coefs_y, tau))

            grad = ca.vertcat(self.poly_deval_nth(coefs_x, tau, 1), self.poly_deval_nth(coefs_y, tau, 1)) / self.spline_info.spline_length
            hessn = ca.vertcat(self.poly_deval_nth(coefs_x, tau, 2), self.poly_deval_nth(coefs_y, tau, 2)) / (self.spline_info.spline_length ** 2)

            tngt = ca.atan2(grad[1], grad[0])
            curv = (grad[0] * hessn[1] - grad[1] * hessn[0]) / (ca.power(grad[0] ** 2 + grad[1] ** 2, 1.5))

            self.spline_func = ca.Function("spline_func", [t], [pos, grad, hessn, tngt, curv])
        else:
            splines = []
            for i in range(self.spline_info.dim):
                coefs_i = coeff[self.spline_info.dim * id_low + i, :]
                eval_i = self.poly_eval_horner(coefs_i, tau)
                splines.append(eval_i)
            self.spline_func = ca.Function("spline_func", [t], splines)
        return

    def get_path_length(self):
        return self.spline_info.spline_length

    @staticmethod
    def poly_eval_horner(coefs, tau):
        D = coefs.shape[1] - 1
        y = coefs[0]
        for k in range(1, D + 1):
            y = tau * y + coefs[k]
        return y
    @staticmethod
    def poly_deval_nth(coefs, tau, n):
        D = coefs.shape[1] - 1
        if n == 0:
            return CasadiSpline.poly_eval_horner(coefs, tau)
        if n > D:
            return 0
        deriv_coefs = ca.MX.zeros(1, D)
        for k in range(D):
            deriv_coefs[0, k] = coefs[0, k] * (D - k)
        return CasadiSpline.poly_deval_nth(deriv_coefs, tau, n - 1)

    @staticmethod
    def shift_warm_path_param(state_path_param, Gamma_warm):
        print_debug = False
        if print_debug:
            print("gamma:", state_path_param, ", warm:", Gamma_warm)

        modify = False
        for i in range(10):
            gamma_err = state_path_param - Gamma_warm[0]
            warm_correction = 0
            if gamma_err < -0.5: warm_correction = -1
            elif gamma_err > 0.5: warm_correction = +1

            if warm_correction != 0:
                Gamma_warm += warm_correction
                modify = True
                print("Warm-start gamma +=", warm_correction, end='')
                if print_debug: print(" =>", Gamma_warm)
                print()
            else:
                break

        return modify
