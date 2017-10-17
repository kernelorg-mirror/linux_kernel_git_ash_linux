#include <linux/perf_event.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/magic.h>
#include <linux/mount.h>
#include <linux/fsnotify.h>
#include "internal.h"

static struct file_system_type	*perffs_mount_type;
static struct vfsmount		*perffs_mount;
static bool 			perffs_registered;
static int 			perffs_mount_count;

#define PERFFS_MAX_COLLISIONS 256

static struct dentry *make_dentry(struct perf_event *event,
				  struct task_struct *task,
				  struct dentry *parent)
{
	unsigned int collisions = 0;
	struct dentry *dentry;
	char *name;

retry:
	name = kasprintf(GFP_KERNEL, "%s:%s%d:%x%x.event", event->pmu->name,
			 task ? "task-" : "cpu",
			 task ? task_pid_nr_ns(task, event->ns) : event->cpu,
			 collisions, hash_64((u64)event, PERFFS_HASH_BITS));
	if (!name)
		return ERR_PTR(-ENOMEM);

	dentry = lookup_one_len(name, parent, strlen(name));
	kfree(name);

	if (!IS_ERR(dentry) && dentry->d_inode) {
		pr_warn("# collision '%s', retrying\n", dentry->d_name.name);
		dput(dentry);

		if (++collisions == PERFFS_MAX_COLLISIONS)
			return ERR_PTR(-EBUSY);

		goto retry;
	}

	return dentry;
}

static struct inode *perffs_get_inode(struct super_block *sb)
{
	struct inode *inode = new_inode(sb);
	if (inode) {
		inode->i_ino = get_next_ino();
		inode->i_atime = inode->i_mtime = inode->i_ctime = current_time(inode);
	}
	return inode;
}

static struct dentry *perffs_create(struct perf_event *event,
				    struct task_struct *task,
				    const struct file_operations *fops)
{
	struct dentry *dentry;
	struct inode *inode;
	struct user_struct *user = current_user();
	int err;

	err = simple_pin_fs(perffs_mount_type, &perffs_mount,
			    &perffs_mount_count);
	if (err)
		return NULL;

	inode_lock(perffs_mount->mnt_root->d_inode);
	dentry = make_dentry(event, task, perffs_mount->mnt_root);
	inode_unlock(perffs_mount->mnt_root->d_inode);
	if (IS_ERR(dentry))
		goto err_release_fs;

	inode = perffs_get_inode(dentry->d_sb);
	if (unlikely(!inode))
		goto err_dput;

	inode->i_mode	= S_IFREG | S_IRUSR | S_IWUSR;
	inode->i_uid	= user->uid;
	inode->i_fop	= fops;
	inode->i_private= event;

	d_instantiate(dentry, inode);
	fsnotify_create(dentry->d_parent->d_inode, dentry);

	return dentry;

err_dput:
	dput(dentry);
err_release_fs:
	simple_release_fs(&perffs_mount, &perffs_mount_count);

	return NULL;
}

int perffs_create_event_file(struct perf_event *event,
			     struct task_struct *task,
			     const struct file_operations *fops)
{
	event->dent = perffs_create(event, task, fops);
	if (!event->dent)
		return -ENOMEM;

	return 0;
}

void perffs_remove(struct dentry *dentry)
{
	struct inode *inode = dentry->d_parent->d_inode;

	inode_lock(inode);
	dget(dentry); /* XXX: why? */
	simple_unlink(inode, dentry);
	d_delete(dentry);
	dput(dentry);
	inode_unlock(inode);
	simple_release_fs(&perffs_mount, &perffs_mount_count);
}

static int perffs_syscall_unlink(struct inode *dir, struct dentry *dentry)
{
	struct inode *inode = dentry->d_inode;
	struct perf_event *event = inode->i_private;

	if (!(event->attach_state & PERF_ATTACH_CONTEXT))
		return -EBUSY;

	inode_unlock(dir);
	inode_unlock(inode);

	perf_event_release_kernel(event);

	inode_lock_nested(dir, I_MUTEX_PARENT);
	inode_lock(inode);

	return 0;
}

static const struct inode_operations perffs_dir_inode_operations = {
	.lookup		= simple_lookup,
	.unlink		= perffs_syscall_unlink,
};

static const struct super_operations perffs_super_operations = {
	.statfs		= simple_statfs,
};

static int perf_fill_super(struct super_block *sb, void *data, int silent)
{
	static const struct tree_descr trace_files[] = {{""}};
	int err;

	err = simple_fill_super(sb, PERFFS_MAGIC, trace_files);
	if (err)
		goto fail;

	sb->s_op = &perffs_super_operations;
	sb->s_root->d_inode->i_op = &perffs_dir_inode_operations;
	sb->s_root->d_inode->i_mode = S_IFDIR | S_IRWXUGO | S_ISVTX;

	return 0;

fail:
	return err;
}

static struct dentry *perf_mount(struct file_system_type *fs_type,
			int flags, const char *dev_name,
			void *data)
{
	return mount_single(fs_type, flags, data, perf_fill_super);
}

static struct file_system_type perf_fs_type = {
	.owner =	THIS_MODULE,
	.name =		"perffs",
	.mount =	perf_mount,
	.kill_sb =	kill_litter_super,
};
//MODULE_ALIAS_FS("perffs");

static int __init perffs_init(void)
{
	int retval;

	retval = sysfs_create_mount_point(kernel_kobj, "perf");
	if (retval)
		return -EINVAL;

	retval = register_filesystem(&perf_fs_type);
	if (!retval) {
		perffs_registered = true;
		perffs_mount_type = &perf_fs_type;
	}

	return retval;
}
core_initcall(perffs_init);
