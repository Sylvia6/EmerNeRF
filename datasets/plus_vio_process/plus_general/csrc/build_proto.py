import os
import argparse
import subprocess


def get_file(path, postfixs):
    result = []
    for root, dirs, files in os.walk(path):
        for file in files:
            # print(os.path.splitext(file)[1])
            postfix = os.path.splitext(file)[1]
            if postfix in postfixs:
                result.append(os.path.join(root, file))
    return result


def make(args):
    save_root = args.install
    if not os.path.exists(save_root):
        os.makedirs(save_root)
    proto_files = get_file(args.src_dir, ['.proto'])
    for proto_file in proto_files:
        rel_path = os.path.relpath(proto_file, './proto')
        install_path = os.path.join(save_root)
        if not os.path.exists(install_path):
            os.makedirs(install_path)
        order = "{} --proto_path={} --python_out={} {}".format(args.protoc, args.src_dir, install_path, proto_file)
        subprocess.call(order, shell=True)
        order = "protol --create-package --in-place --python-out={} protoc --protoc-path={} --proto-path={}  {}".format(
            install_path, args.protoc, args.src_dir, proto_file)
        print(order)
        subprocess.call(order, shell=True)


if __name__ == "__main__":
    folder = os.path.dirname(os.path.realpath(__file__))
    common_protobuf_dir = os.path.join(folder, "common_protobuf")
    install_path = os.path.join(folder, "common_proto_py")
    parser = argparse.ArgumentParser()
    parser.add_argument("--protoc", "-p", type=str, default="system", choices=['system', 'torch'])
    parser.add_argument("--install", "-i", type=str, default=install_path)
    args = parser.parse_args()

    if args.protoc == "system":
        args.protoc = "protoc"
    elif args.protoc == "torch":
        import torch
        torch_dir = os.path.dirname(torch.__file__)
        torch_protoc_path = os.path.join(torch_dir, 'bin', 'protoc')
        args.protoc = torch_protoc_path
    else:
        raise NotImplementedError
    args.src_dir = common_protobuf_dir
    make(args)

