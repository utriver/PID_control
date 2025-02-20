import json
import sys

def load_parameters():
    try:
        with open('/home/user/release/friction_Id/friction_parameters.json', 'r') as f:
            data = json.load(f)
        
        params = data['FricParameter'][0]
        
        # 기본 파라미터 출력
        print(f"{params['F_c_n']},{params['F_c_p']},"
              f"{params['F_s_n']},{params['F_s_p']},"
              f"{params['F_v_n']},{params['F_v_p']},"
              f"{params['delta_n']},{params['delta_p']},"
              f"{params['sigma_2_n']},{params['sigma_2_p']},"
              f"{params['C']},{params['gamma1']},{params['gamma2']}")
        
        # k 파라미터 출력
        print(','.join(str(x) for x in params['k']))
        
        # alpha 파라미터 출력
        print(','.join(str(x) for x in params['alpha']))

        # sigma 파라미터 출력
        print(','.join(str(x) for x in params['sigma']))
        
    except Exception as e:
        print(f"Error: {str(e)}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    load_parameters()